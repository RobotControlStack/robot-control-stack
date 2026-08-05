"""Convert RCS parquet recordings to an uncompressed, training-ready LMDB.

This backend deliberately reuses :class:`JointDatasetConverter`: episode
filtering, optional repeated-action removal, optional gripper binarization,
Cartesian to joint-space IK, camera selection, JPEG decoding and resizing are
therefore the same as for the LeRobot export.  The only difference is the frame
sink.

Images are stored as contiguous uint8 CHW arrays.  They are decoded from the
source JPEG and resized once while converting; the training dataloader only
copies bytes out of LMDB and never invokes a video or image decoder.
"""
from __future__ import annotations

from contextlib import suppress
from pathlib import Path
from typing import Any

import numpy as np

from rcs._core.common import GripperType, RobotType
from rcs.lerobot_joint_converter import (
    DEFAULT_BINARIZE_GRIPPER,
    DEFAULT_CAMERAS,
    DEFAULT_DATASET_PATHS,
    DEFAULT_FPS,
    DEFAULT_GRIPPER_BINARIZE_THRESHOLD,
    DEFAULT_GRIPPER_TYPE,
    DEFAULT_IMAGE_BATCH_SIZE,
    DEFAULT_JOINTS,
    DEFAULT_PER_ROBOT_ARM_DIM,
    DEFAULT_REPO_ID,
    DEFAULT_ROBOT_KEYS,
    DEFAULT_ROBOT_TYPE,
    CamConversionConfig,
    JointDatasetConverter,
)

DEFAULT_LMDB_DATA_DIR = "data_lmdb_joint_simple.lmdb"
DEFAULT_MAP_SIZE_GB = 4
DEFAULT_WRITE_BATCH_SIZE = 128
DEFAULT_AUTO_GROW = True
DEFAULT_SHRINK_TO_FIT = True
_GB = 1024**3
_META_KEY = b"meta"
_NDARRAY_TAG = "__ndarray__"


def _frame_key(index: int) -> bytes:
    return f"frame/{index:09d}".encode("ascii")


def _msgpack_default(value: Any) -> dict[str, Any]:
    if isinstance(value, np.ndarray):
        array = value if value.flags.c_contiguous else np.ascontiguousarray(value)
        return {
            _NDARRAY_TAG: True,
            "dtype": array.dtype.str,
            "shape": list(array.shape),
            "data": array.tobytes(),
        }
    if isinstance(value, np.generic):
        return value.item()
    msg = f"Cannot serialize {type(value)!r} to LMDB"
    raise TypeError(msg)


def _encode_record(value: dict[str, Any]) -> bytes:
    try:
        import msgpack
    except ImportError as exc:  # pragma: no cover - environment dependent
        msg = "LMDB conversion requires msgpack; install the rcs_core dependencies"
        raise RuntimeError(msg) from exc
    return msgpack.packb(value, default=_msgpack_default, use_bin_type=True)


class _VectorStats:
    """Small exact accumulator for state/action statistics and quantiles."""

    def __init__(self) -> None:
        self.rows: list[np.ndarray] = []

    def update(self, value: np.ndarray) -> None:
        self.rows.append(np.asarray(value, dtype=np.float32).copy())

    def finalize(self) -> dict[str, Any]:
        values = np.stack(self.rows).astype(np.float64, copy=False)
        result: dict[str, Any] = {
            "min": values.min(axis=0).astype(np.float32),
            "max": values.max(axis=0).astype(np.float32),
            "mean": values.mean(axis=0).astype(np.float32),
            "std": values.std(axis=0).astype(np.float32),
            "count": np.asarray([len(values)], dtype=np.int64),
        }
        for name, quantile in (("q01", 0.01), ("q10", 0.10), ("q50", 0.50), ("q90", 0.90), ("q99", 0.99)):
            result[name] = np.quantile(values, quantile, axis=0).astype(np.float32)
        return result


class _ImageStats:
    """Accumulate per-channel pixel statistics without retaining images."""

    def __init__(self) -> None:
        self.pixel_count = 0
        self.frame_count = 0
        self.sum = np.zeros(3, dtype=np.float64)
        self.sum_sq = np.zeros(3, dtype=np.float64)
        self.min = np.full(3, 255, dtype=np.uint8)
        self.max = np.zeros(3, dtype=np.uint8)
        self.histogram = np.zeros((3, 256), dtype=np.int64)

    def update(self, image_chw: np.ndarray) -> None:
        pixels = np.asarray(image_chw, dtype=np.uint8).reshape(3, -1)
        self.pixel_count += pixels.shape[1]
        self.frame_count += 1
        self.sum += pixels.sum(axis=1, dtype=np.float64)
        pixels_f32 = pixels.astype(np.float32)
        self.sum_sq += np.square(pixels_f32).sum(axis=1, dtype=np.float64)
        self.min = np.minimum(self.min, pixels.min(axis=1))
        self.max = np.maximum(self.max, pixels.max(axis=1))
        for channel in range(3):
            self.histogram[channel] += np.bincount(pixels[channel], minlength=256)

    def finalize(self) -> dict[str, Any]:
        scale = 1.0 / 255.0
        mean_u8 = self.sum / self.pixel_count
        variance_u8 = np.maximum(self.sum_sq / self.pixel_count - np.square(mean_u8), 0.0)

        def channel_shape(value: np.ndarray) -> np.ndarray:
            return np.asarray(value, dtype=np.float32).reshape(3, 1, 1)

        result: dict[str, Any] = {
            "min": channel_shape(self.min * scale),
            "max": channel_shape(self.max * scale),
            "mean": channel_shape(mean_u8 * scale),
            "std": channel_shape(np.sqrt(variance_u8) * scale),
            "count": np.asarray([self.frame_count], dtype=np.int64),
        }
        cumulative = self.histogram.cumsum(axis=1)
        for name, quantile in (("q01", 0.01), ("q10", 0.10), ("q50", 0.50), ("q90", 0.90), ("q99", 0.99)):
            targets = np.ceil(quantile * self.pixel_count).astype(np.int64)
            values = np.asarray(
                [np.searchsorted(cumulative[channel], targets) for channel in range(3)]
            )
            result[name] = channel_shape(values * scale)
        return result


class LMDBFrameWriter:
    """LeRobot-like frame sink backed by one memory-mapped LMDB file."""

    def __init__(
        self,
        *,
        root: str | Path,
        repo_id: str,
        robot_type: str,
        fps: int,
        features: dict[str, dict[str, Any]],
        map_size_gb: int = DEFAULT_MAP_SIZE_GB,
        write_batch_size: int = DEFAULT_WRITE_BATCH_SIZE,
        auto_grow: bool = DEFAULT_AUTO_GROW,
        shrink_to_fit: bool = DEFAULT_SHRINK_TO_FIT,
        overwrite: bool = False,
    ) -> None:
        try:
            import lmdb
        except ImportError as exc:  # pragma: no cover - environment dependent
            msg = "LMDB conversion requires lmdb; install the rcs_core dependencies"
            raise RuntimeError(msg) from exc

        if map_size_gb <= 0:
            msg = f"map_size_gb must be positive, got {map_size_gb}"
            raise ValueError(msg)
        if write_batch_size <= 0:
            msg = f"write_batch_size must be positive, got {write_batch_size}"
            raise ValueError(msg)

        self._lmdb = lmdb
        self.root = Path(root).expanduser().resolve()
        self.root.parent.mkdir(parents=True, exist_ok=True)
        lock_path = Path(f"{self.root}-lock")
        if self.root.exists():
            if not overwrite:
                msg = f"LMDB output already exists: {self.root} (pass overwrite=True to replace it)"
                raise FileExistsError(msg)
            if not self.root.is_file():
                msg = f"Expected a single-file LMDB output, found directory: {self.root}"
                raise IsADirectoryError(msg)
            self.root.unlink()
        if overwrite and lock_path.exists():
            lock_path.unlink()

        self.repo_id = repo_id
        self.robot_type = robot_type
        self.fps = int(fps)
        self.features = features
        self.write_batch_size = int(write_batch_size)
        self.auto_grow = bool(auto_grow)
        self.shrink_to_fit = bool(shrink_to_fit)
        self.env = lmdb.open(
            str(self.root),
            subdir=False,
            map_size=int(map_size_gb * _GB),
            writemap=True,
            map_async=True,
            metasync=False,
            sync=False,
        )
        self.txn = self.env.begin(write=True)
        self._pending_writes: list[tuple[bytes, bytes]] = []
        self.total_frames = 0
        self.episode_frame_index = 0
        self.episode_from_indices: list[int] = []
        self.episode_to_indices: list[int] = []
        self.episode_lengths: list[int] = []
        self.episode_tasks: list[str] = []
        self.tasks: list[str] = []
        self.task_to_index: dict[str, int] = {}
        self.actions: list[np.ndarray] = []
        self.state_stats = _VectorStats()
        self.action_stats = _VectorStats()
        self.image_stats = {
            key: _ImageStats()
            for key, feature in self.features.items()
            if feature.get("dtype") == "image"
        }
        self._closed = False

    @property
    def episode_index(self) -> int:
        return len(self.episode_lengths)

    def _grow_map(self) -> None:
        current_size = int(self.env.info()["map_size"])
        new_size = max(current_size * 2, current_size + _GB)
        self.env.set_mapsize(new_size)

    def _replay_pending_after_growth(self) -> None:
        if not self.auto_grow:
            msg = "LMDB map is full; increase --map-size-gb or enable --auto-grow"
            raise self._lmdb.MapFullError(msg)
        if self.txn is not None:
            with suppress(self._lmdb.Error):
                self.txn.abort()
            self.txn = None

        while True:
            self._grow_map()
            transaction = self.env.begin(write=True)
            try:
                for key, value in self._pending_writes:
                    transaction.put(key, value)
            except self._lmdb.MapFullError:
                transaction.abort()
                continue
            self.txn = transaction
            return

    def _put(self, key: bytes, value: bytes) -> None:
        self._pending_writes.append((key, value))
        assert self.txn is not None
        try:
            self.txn.put(key, value)
        except self._lmdb.MapFullError:
            self._replay_pending_after_growth()

    def _commit(self, *, begin_next: bool) -> None:
        assert self.txn is not None
        try:
            self.txn.commit()
        except self._lmdb.MapFullError:
            self._replay_pending_after_growth()
            assert self.txn is not None
            self.txn.commit()
        self.txn = self.env.begin(write=True) if begin_next else None
        self._pending_writes.clear()

    def _commit_if_needed(self) -> None:
        if self.total_frames > 0 and self.total_frames % self.write_batch_size == 0:
            self._commit(begin_next=True)

    def _shrink_map_to_fit(self) -> None:
        info = self.env.info()
        page_size = int(self.env.stat()["psize"])
        # The dataset is immutable after finalize(), so the last allocated LMDB
        # page is also the required final map boundary. No growth headroom is
        # needed; a future dataset version is written to a fresh file.
        target_size = (int(info["last_pgno"]) + 1) * page_size
        if target_size < int(info["map_size"]):
            self.env.set_mapsize(target_size)

    def add_frame(self, frame: dict[str, Any]) -> None:
        record: dict[str, Any] = {}
        for key in self.image_stats:
            image_hwc = np.asarray(frame[key], dtype=np.uint8)
            expected_h, expected_w, expected_c = self.features[key]["shape"]
            if image_hwc.shape != (expected_h, expected_w, expected_c):
                msg = f"Unexpected image shape for {key}: {image_hwc.shape}"
                raise ValueError(msg)
            image_chw = np.ascontiguousarray(image_hwc.transpose(2, 0, 1))
            record[key] = image_chw
            self.image_stats[key].update(image_chw)

        state = np.asarray(frame["observation.state"], dtype=np.float32)
        action = np.asarray(frame["action"], dtype=np.float32)
        self.state_stats.update(state)
        self.action_stats.update(action)
        self.actions.append(action.copy())

        task = str(frame.get("task", ""))
        if task not in self.task_to_index:
            self.task_to_index[task] = len(self.tasks)
            self.tasks.append(task)
        if self.episode_frame_index == 0:
            self.episode_from_indices.append(self.total_frames)
            self.episode_tasks.append(task)

        record.update(
            {
                "observation.state": state,
                "action": action,
                "task": task,
                "timestamp": np.asarray(self.episode_frame_index / self.fps, dtype=np.float32),
                "frame_index": self.episode_frame_index,
                "episode_index": self.episode_index,
                "index": self.total_frames,
                "task_index": self.task_to_index[task],
            }
        )
        self._put(_frame_key(self.total_frames), _encode_record(record))
        self.total_frames += 1
        self.episode_frame_index += 1
        self._commit_if_needed()

    def save_episode(self) -> None:
        if self.episode_frame_index <= 0:
            msg = "Cannot save an empty LMDB episode"
            raise RuntimeError(msg)
        self.episode_lengths.append(self.episode_frame_index)
        self.episode_to_indices.append(self.total_frames)
        self.episode_frame_index = 0

    def finalize(self) -> None:
        if self.episode_frame_index:
            msg = "The final LMDB episode was not saved"
            raise RuntimeError(msg)
        if not self.total_frames:
            msg = "No frames were written to the LMDB dataset"
            raise RuntimeError(msg)

        stats: dict[str, Any] = {
            "observation.state": self.state_stats.finalize(),
            "action": self.action_stats.finalize(),
        }
        stats.update({key: value.finalize() for key, value in self.image_stats.items()})
        meta = {
            "format": "rcs.lmdb_joint.v1",
            "repo_id": self.repo_id,
            "robot_type": self.robot_type,
            "fps": self.fps,
            "total_frames": self.total_frames,
            "total_episodes": len(self.episode_lengths),
            "features": self.features,
            "stats": stats,
            "episode_lengths": self.episode_lengths,
            "episode_from_indices": self.episode_from_indices,
            "episode_to_indices": self.episode_to_indices,
            "episode_tasks": self.episode_tasks,
            "tasks": self.tasks,
            "task_indices": [self.task_to_index[task] for task in self.episode_tasks],
            "image_layout": "CHW",
            # Actions are tiny relative to images and keeping them here makes an
            # action chunk a memory slice instead of H extra LMDB transactions.
            "actions": np.stack(self.actions).astype(np.float32, copy=False),
        }
        self._put(_META_KEY, _encode_record(meta))
        self._commit(begin_next=False)
        self.env.sync(True)
        if self.shrink_to_fit:
            self._shrink_map_to_fit()
            self.env.sync(True)
        self.env.close()
        self._closed = True

    def close(self) -> None:
        if self._closed:
            return
        if self.txn is not None:
            self.txn.abort()
            self.txn = None
        self.env.close()
        self._closed = True

    def __del__(self) -> None:  # pragma: no cover - best effort after exceptions
        try:  # noqa: SIM105
            self.close()
        except Exception:
            pass


class LMDBJointDatasetConverter(JointDatasetConverter):
    """Joint converter using :class:`LMDBFrameWriter` as its output sink."""

    def __init__(
        self,
        *args: Any,
        map_size_gb: int = DEFAULT_MAP_SIZE_GB,
        write_batch_size: int = DEFAULT_WRITE_BATCH_SIZE,
        auto_grow: bool = DEFAULT_AUTO_GROW,
        shrink_to_fit: bool = DEFAULT_SHRINK_TO_FIT,
        overwrite: bool = False,
        **kwargs: Any,
    ) -> None:
        self.map_size_gb = map_size_gb
        self.write_batch_size = write_batch_size
        self.auto_grow = auto_grow
        self.shrink_to_fit = shrink_to_fit
        self.overwrite = overwrite
        if kwargs.get("video_encoding"):
            msg = "The LMDB backend stores decoded uint8 images and does not support video encoding"
            raise ValueError(msg)
        kwargs["video_encoding"] = False
        kwargs.pop("video_backend", None)
        super().__init__(*args, **kwargs)

    def _create_output_dataset(self, video_backend: str | None = None) -> LMDBFrameWriter:
        del video_backend
        return LMDBFrameWriter(
            root=self.root,
            repo_id=self.repo_id,
            robot_type=self.robot_type.id,
            fps=self.fps,
            features=self._build_features(),
            map_size_gb=self.map_size_gb,
            write_batch_size=self.write_batch_size,
            auto_grow=self.auto_grow,
            shrink_to_fit=self.shrink_to_fit,
            overwrite=self.overwrite,
        )


def run_conversion(
    root: str | Path = DEFAULT_LMDB_DATA_DIR,
    dataset_paths: list[str] | None = None,
    repo_id: str = DEFAULT_REPO_ID,
    robot_type: str = DEFAULT_ROBOT_TYPE,
    fps: int = DEFAULT_FPS,
    robot_keys: list[str] | None = None,
    joints: bool = DEFAULT_JOINTS,
    gripper_type: str = DEFAULT_GRIPPER_TYPE,
    cameras: list[CamConversionConfig] | None = None,
    image_batch_size: int = DEFAULT_IMAGE_BATCH_SIZE,
    per_robot_arm_dim: int = DEFAULT_PER_ROBOT_ARM_DIM,
    binarize_gripper: bool = DEFAULT_BINARIZE_GRIPPER,
    gripper_binarize_threshold: float = DEFAULT_GRIPPER_BINARIZE_THRESHOLD,
    success: bool = True,
    n: int = -1,
    map_size_gb: int = DEFAULT_MAP_SIZE_GB,
    write_batch_size: int = DEFAULT_WRITE_BATCH_SIZE,
    auto_grow: bool = DEFAULT_AUTO_GROW,
    shrink_to_fit: bool = DEFAULT_SHRINK_TO_FIT,
    overwrite: bool = False,
    disable_stationary_frame_filtering: bool = False,
) -> None:
    converter = LMDBJointDatasetConverter(
        root=root,
        robot_type=RobotType(robot_type),
        gripper_type=GripperType(gripper_type),
        dataset_paths=dataset_paths or list(DEFAULT_DATASET_PATHS),
        repo_id=repo_id,
        fps=fps,
        robot_keys=robot_keys or list(DEFAULT_ROBOT_KEYS),
        joints=joints,
        cameras=cameras or list(DEFAULT_CAMERAS),
        image_batch_size=image_batch_size,
        per_robot_arm_dim=per_robot_arm_dim,
        binarize_gripper=binarize_gripper,
        gripper_binarize_threshold=gripper_binarize_threshold,
        disable_stationary_frame_filtering=disable_stationary_frame_filtering,
        map_size_gb=map_size_gb,
        write_batch_size=write_batch_size,
        auto_grow=auto_grow,
        shrink_to_fit=shrink_to_fit,
        overwrite=overwrite,
    )
    try:
        converter.generate_examples(success=success, n=n)
    except Exception:
        converter.lrds.close()
        raise


if __name__ == "__main__":
    run_conversion()
