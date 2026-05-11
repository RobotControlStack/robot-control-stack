from __future__ import annotations

import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import duckdb
import numpy as np
import pandas as pd
import pyarrow as pa
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from rcs._core.common import GripperType, RobotType
from torchvision.io import decode_jpeg
from torchvision.transforms import v2

import rcs

DEFAULT_DATASET_PATHS = [
    "data_grasp",
]
DEFAULT_HF_DATA_DIR = "data_lerobot_joint_simple"
DEFAULT_REPO_ID = "rcs/grasp_joint_simple"
DEFAULT_ROBOT_TYPE = "FR3"
DEFAULT_FPS = 30
DEFAULT_ROBOT_KEYS = ["left", "right"]
DEFAULT_JOINTS = False
DEFAULT_GRIPPER_TYPE = "Robotiq2F85"
DEFAULT_BINARIZE_GRIPPER = False
DEFAULT_GRIPPER_BINARIZE_THRESHOLD = 0.2


@dataclass(frozen=True)
class CamConversionConfig:
    name: str
    resolution: tuple[int, int]
    source_name: str | None = None

    @property
    def dataset_key(self) -> str:
        return f"observation.images.{self.name}"

    @property
    def frame_name(self) -> str:
        return self.source_name or self.name.removeprefix("image_")

    @property
    def image_column(self) -> str:
        return f"image_{self.name}"


DEFAULT_CAMERAS = [
    CamConversionConfig(name="head", resolution=(256, 256)),
    CamConversionConfig(name="image_left_wrist", source_name="left_wrist", resolution=(256, 256)),
    CamConversionConfig(name="image_right_wrist", source_name="right_wrist", resolution=(256, 256)),
]
DEFAULT_IMAGE_BATCH_SIZE = 32
DEFAULT_PER_ROBOT_ARM_DIM = 7


def parse_camera_spec(spec: str) -> CamConversionConfig:
    name_source, _, resolution_spec = spec.partition("@")
    name, sep, source_name = name_source.partition(":")
    if not name:
        msg = f"Invalid camera spec '{spec}'"
        raise ValueError(msg)

    resolution = (256, 256)
    if resolution_spec:
        try:
            height_str, width_str = resolution_spec.lower().split("x", maxsplit=1)
            resolution = (int(height_str), int(width_str))
        except ValueError as exc:
            msg = f"Invalid camera resolution in spec '{spec}'"
            raise ValueError(msg) from exc

    return CamConversionConfig(
        name=name,
        source_name=source_name or None if sep else None,
        resolution=resolution,
    )


def camera_specs_to_configs(camera_specs: Iterable[str]) -> list[CamConversionConfig]:
    return [parse_camera_spec(spec) for spec in camera_specs]


class JointDatasetConverter:
    def __init__(
        self,
        root: str | Path,
        robot_type: RobotType,
        gripper_type: GripperType,
        dataset_paths: list[str] | None = None,
        repo_id: str = DEFAULT_REPO_ID,
        fps: int = DEFAULT_FPS,
        robot_keys: list[str] | None = None,
        joints: bool = DEFAULT_JOINTS,
        cameras: list[CamConversionConfig] | None = None,
        image_batch_size: int = DEFAULT_IMAGE_BATCH_SIZE,
        per_robot_arm_dim: int = DEFAULT_PER_ROBOT_ARM_DIM,
        binarize_gripper: bool = DEFAULT_BINARIZE_GRIPPER,
        gripper_binarize_threshold: float = DEFAULT_GRIPPER_BINARIZE_THRESHOLD,
        video_encoding: bool = False,
        video_backend: str | None = None,
    ):
        self.root = Path(root)
        self.conn = duckdb.connect()
        self.dataset_paths = dataset_paths or list(DEFAULT_DATASET_PATHS)
        self.repo_id = repo_id
        self.robot_type = robot_type
        self.fps = fps
        self.robot_keys = robot_keys or list(DEFAULT_ROBOT_KEYS)
        self.joints = joints
        self.gripper_type = gripper_type
        self.cameras = cameras or list(DEFAULT_CAMERAS)
        self.image_batch_size = image_batch_size
        self.per_robot_arm_dim = per_robot_arm_dim
        self.per_robot_state_dim = self.per_robot_arm_dim + 1
        self.state_dim = len(self.robot_keys) * self.per_robot_state_dim
        self.binarize_gripper = binarize_gripper
        self.gripper_binarize_threshold = gripper_binarize_threshold
        self.source_sql = self._build_source_sql(self.dataset_paths)
        self.video_encoding = video_encoding

        self.tcp_offset = rcs.GRIPPER_OFFSETS[self.gripper_type]
        self.ik = rcs.common.Pin(
            rcs.ROBOTS[robot_type].mjcf_model_path,
            rcs.ROBOTS[robot_type].attachment_site,
        )
        self.camera_resizers = {camera.name: v2.Resize(camera.resolution) for camera in self.cameras}

        self.lrds = LeRobotDataset.create(
            repo_id=self.repo_id,
            robot_type=self.robot_type.id,
            root=self.root,
            fps=self.fps,
            use_videos=self.video_encoding,
            features=self._build_features(),
            image_writer_threads=10,
            image_writer_processes=5,
            video_backend=video_backend,
        )

    def _maybe_binarize_gripper(self, gripper: np.ndarray) -> np.ndarray:
        if not self.binarize_gripper:
            return gripper.astype(np.float32)
        return (gripper > self.gripper_binarize_threshold).astype(np.float32)

    def _build_features(self) -> dict[str, dict[str, Any]]:
        state_names = []
        for robot_key in self.robot_keys:
            state_names.extend([f"{robot_key}_joint_{i}" for i in range(self.per_robot_arm_dim)])
            state_names.append(f"{robot_key}_gripper")

        features = {
            camera.dataset_key: {
                "dtype": "video" if self.video_encoding else "image",
                "shape": (*camera.resolution, 3),
                "names": ["height", "width", "channel"],
            }
            for camera in self.cameras
        }
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (self.state_dim,),
            "names": state_names,
        }
        features["action"] = {
            "dtype": "float32",
            "shape": (self.state_dim,),
            "names": state_names,
        }
        return features

    def _build_source_sql(self, dataset_paths: list[str]) -> str:
        queries = []
        for path in dataset_paths:
            escaped = str(path).replace("'", "''")
            queries.append(f"SELECT * FROM read_parquet('{escaped}')")
        return " UNION ALL ".join(queries)

    def generate_examples(self, success: bool = True, n: int = -1):
        uuids = self.conn.execute(f"SELECT DISTINCT uuid FROM ({self.source_sql}) AS src ORDER BY uuid").fetchall()

        for (episode_id,) in uuids:
            table = self._fetch_transition_table(episode_id)

            converted = self.parse_episode(episode_id, table, success)
            if converted:
                n -= 1
                if n == 0:
                    break

        self.lrds.finalize()

    def _fetch_transition_table(self, episode_id: str) -> pd.DataFrame:
        observation_selects = ",\n                    ".join(
            [f"obs.{robot_key}.joints AS observation_joints_{robot_key}" for robot_key in self.robot_keys]
            + [f"obs.{robot_key}.gripper AS observation_gripper_{robot_key}" for robot_key in self.robot_keys]
        )
        action_selects = ",\n                    ".join(
            [f"info.{robot_key}.absolute_action AS absolute_action_{robot_key}" for robot_key in self.robot_keys]
            + [f"env_action.{robot_key}.gripper AS action_gripper_{robot_key}" for robot_key in self.robot_keys]
        )

        return self.conn.execute(
            f"""
            SELECT
                uuid,
                step,
                success,
                instruction,
                {observation_selects},
                {action_selects}
            FROM ({self.source_sql}) AS src
            WHERE uuid = ?
            ORDER BY step
            """,
            [episode_id],
        ).df()

    def _fetch_episode_success(self, episode_id: str) -> bool:
        success = self.conn.execute(
            f"SELECT COALESCE(MAX(success), FALSE) FROM ({self.source_sql}) AS src WHERE uuid = ?",
            [episode_id],
        ).fetchone()
        assert success is not None
        return bool(success[0])

    def _image_query(self) -> str:
        image_selects = ",\n                    ".join(
            f"obs.frames.{camera.frame_name}.rgb.data AS {camera.image_column}" for camera in self.cameras
        )
        image_not_null_checks = "\n                  ".join(
            f"AND obs.frames.{camera.frame_name}.rgb.data IS NOT NULL" for camera in self.cameras
        )
        image_columns = ",\n                ".join(camera.image_column for camera in self.cameras)

        return f"""
            WITH ordered AS (
                SELECT
                    uuid,
                    step,
                    {image_selects}
                FROM ({self.source_sql}) AS src
                WHERE uuid = ?
                  {image_not_null_checks}
            )
            SELECT
                step,
                {image_columns}
            FROM ordered
            ORDER BY step
        """

    def _is_missing(self, value: object) -> bool:
        if value is None or value is pd.NA:
            return True
        if isinstance(value, float):
            return bool(np.isnan(value))
        return False

    def _build_observation_state(self, row: pd.Series) -> np.ndarray:
        vectors = []
        for robot_key in self.robot_keys:
            joints = row[f"observation_joints_{robot_key}"]
            gripper = row[f"observation_gripper_{robot_key}"]
            if self._is_missing(joints) or self._is_missing(gripper):
                msg = f"Missing observation state for robot '{robot_key}' at step {row['step']}"
                raise ValueError(msg)

            joints_vec = np.asarray(joints, dtype=np.float32)
            gripper_vec = np.asarray(gripper, dtype=np.float32)
            if joints_vec.shape != (self.per_robot_arm_dim,) or gripper_vec.shape != (1,):
                msg = (
                    f"Unexpected observation shapes for robot '{robot_key}' at step {row['step']}: "
                    f"joints={joints_vec.shape}, gripper={gripper_vec.shape}"
                )
                raise ValueError(msg)
            gripper_vec = self._maybe_binarize_gripper(gripper_vec)
            vectors.append(np.concatenate([joints_vec, gripper_vec]).astype(np.float32))

        return np.concatenate(vectors).astype(np.float32)

    def _convert_action_to_joint_space(self, row: pd.Series) -> np.ndarray | None:
        actions = []
        for robot_key in self.robot_keys:
            observation_joints = row[f"observation_joints_{robot_key}"]
            absolute_action = row[f"absolute_action_{robot_key}"]
            action_gripper = row[f"action_gripper_{robot_key}"]
            if (
                self._is_missing(observation_joints)
                or self._is_missing(absolute_action)
                or self._is_missing(action_gripper)
            ):
                msg = f"Missing action inputs for robot '{robot_key}' at step {row['step']}"
                raise ValueError(msg)

            observation_joints_vec = np.asarray(observation_joints, dtype=np.float64)
            absolute_action_vec = np.asarray(absolute_action, dtype=np.float64)
            action_gripper_vec = np.asarray(action_gripper, dtype=np.float32)
            if (
                observation_joints_vec.shape != (self.per_robot_arm_dim,)
                or absolute_action_vec.shape != (self.per_robot_arm_dim,)
                or action_gripper_vec.shape != (1,)
            ):
                msg = (
                    f"Unexpected action shapes for robot '{robot_key}' at step {row['step']}: "
                    f"observation_joints={observation_joints_vec.shape}, "
                    f"absolute_action={absolute_action_vec.shape}, action_gripper={action_gripper_vec.shape}"
                )
                raise ValueError(msg)
            action_gripper_vec = self._maybe_binarize_gripper(action_gripper_vec)

            if self.joints:
                arm_action_vec = absolute_action_vec.astype(np.float32)
            else:
                target_pose = rcs.common.Pose(
                    translation=absolute_action_vec[:3],
                    quaternion=absolute_action_vec[3:7],
                )
                ik_joints: np.ndarray | None = self.ik.inverse(
                    target_pose, observation_joints_vec, tcp_offset=self.tcp_offset
                )
                if ik_joints is None:
                    msg = f"IK failed for robot '{robot_key}' at step {row['step']}, ignoring step"
                    warnings.warn(msg, stacklevel=1)
                    return None
                arm_action_vec = np.asarray(ik_joints, dtype=np.float32)

            actions.append(np.concatenate([arm_action_vec, action_gripper_vec]).astype(np.float32))

        concatenated = np.concatenate(actions).astype(np.float32)
        if concatenated.shape != (self.state_dim,):
            msg = f"Unexpected concatenated action shape {concatenated.shape} at step {row['step']}"
            raise ValueError(msg)
        return concatenated

    def _prepare_transition_table(self, table: pd.DataFrame) -> pd.DataFrame:
        if len(table) == 0:
            return table

        df = table.copy()  # noqa: PD901
        df["observation_state"] = df.apply(self._build_observation_state, axis=1)
        df["action_vector"] = df.apply(self._convert_action_to_joint_space, axis=1)

        df = df[df["action_vector"].notna()]  # noqa: PD901

        prev_action: np.ndarray | None = None
        keep_mask = []
        for action_vec in df["action_vector"]:
            assert isinstance(action_vec, np.ndarray)
            keep_mask.append(prev_action is None or not np.allclose(action_vec, prev_action, atol=1e-4, rtol=0))
            prev_action = action_vec

        return df.loc[keep_mask].reset_index(drop=True)

    def parse_episode(self, episode_id: str, table: pd.DataFrame, success: bool):
        table = self._prepare_transition_table(table)
        if len(table) == 0:
            return False

        if success and not self._fetch_episode_success(episode_id):
            return False

        df = table.reset_index(drop=True)  # noqa: PD901
        rows_by_step = {int(row["step"]): row for _, row in df.iterrows()}
        step_order = [int(step) for step in df["step"].tolist()]
        frames_by_step: dict[int, dict[str, np.ndarray]] = {}

        reader = self.conn.execute(self._image_query(), [episode_id]).fetch_record_batch(
            rows_per_batch=self.image_batch_size
        )
        for batch in reader:
            self._decode_image_batch(batch, frames_by_step)

        num_frames_added = 0
        for step in step_order:
            curr = rows_by_step[step]
            if step not in frames_by_step:
                continue
            images = frames_by_step[step]

            frame: dict[str, Any] = {camera.dataset_key: images[camera.name] for camera in self.cameras}
            frame["observation.state"] = curr["observation_state"]
            frame["action"] = curr["action_vector"]
            frame["task"] = str(curr["instruction"])

            self.lrds.add_frame(frame)
            num_frames_added += 1

        if num_frames_added == 0:
            return False
        self.lrds.save_episode()
        return True

    def _decode_and_resize_batch(self, image_bytes_list: list[bytes], camera: CamConversionConfig) -> np.ndarray:
        image_tensors = [
            torch.frombuffer(bytearray(image_bytes), dtype=torch.uint8) for image_bytes in image_bytes_list
        ]
        decoded = decode_jpeg(image_tensors)
        batch = torch.stack(decoded)
        resized = self.camera_resizers[camera.name](batch)
        return resized.permute(0, 2, 3, 1).cpu().numpy()

    def _decode_image_batch(self, batch: pa.RecordBatch, frames_by_step: dict[int, dict[str, np.ndarray]]) -> None:
        batch_dict = batch.to_pydict()
        steps = [int(step) for step in batch_dict["step"]]
        decoded_images = {}
        for camera in self.cameras:
            decoded_images[camera.name] = self._decode_and_resize_batch(batch_dict[camera.image_column], camera)

        for idx, step in enumerate(steps):
            frames_by_step[step] = {camera.name: decoded_images[camera.name][idx] for camera in self.cameras}


def run_conversion(
    root: str | Path = DEFAULT_HF_DATA_DIR,
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
    video_encoding: bool = False,
    video_backend: str | None = None,
) -> None:
    robot_type_converted = RobotType(robot_type)
    gripper_type_converted = GripperType(gripper_type)
    converter = JointDatasetConverter(
        root=root,
        robot_type=robot_type_converted,
        gripper_type=gripper_type_converted,
        dataset_paths=dataset_paths,
        repo_id=repo_id,
        fps=fps,
        robot_keys=robot_keys,
        joints=joints,
        cameras=cameras,
        image_batch_size=image_batch_size,
        per_robot_arm_dim=per_robot_arm_dim,
        binarize_gripper=binarize_gripper,
        gripper_binarize_threshold=gripper_binarize_threshold,
        video_encoding=video_encoding,
        video_backend=video_backend,
    )
    converter.generate_examples(success=success, n=n)


if __name__ == "__main__":
    run_conversion()
