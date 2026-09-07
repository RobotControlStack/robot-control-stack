from __future__ import annotations

import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import duckdb
import numpy as np
import pandas as pd
import pyarrow as pa
from rcs._core.common import GripperType, RobotType
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
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
DEFAULT_SOURCE_ACTION_IS_JOINT = False
DEFAULT_ACTION_SOURCE_FIELD = "absolute_action"
DEFAULT_RETURNED_STATE_TYPE = "joints"
DEFAULT_RETURNED_ACTION_TYPE = "tquat"
DEFAULT_DELTA_FROM_OBSERVATION = False
DEFAULT_GRIPPER_TYPE = "Robotiq2F85"
DEFAULT_BINARIZE_GRIPPER = False
DEFAULT_GRIPPER_BINARIZE_THRESHOLD = 0.9


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
        source_action_is_joint: bool = DEFAULT_SOURCE_ACTION_IS_JOINT,
        action_source_field: str = DEFAULT_ACTION_SOURCE_FIELD,
        returned_state_type: str = DEFAULT_RETURNED_STATE_TYPE,
        returned_action_type: str = DEFAULT_RETURNED_ACTION_TYPE,
        delta_from_observation: bool = DEFAULT_DELTA_FROM_OBSERVATION,
        cameras: list[CamConversionConfig] | None = None,
        image_batch_size: int = DEFAULT_IMAGE_BATCH_SIZE,
        per_robot_arm_dim: int = DEFAULT_PER_ROBOT_ARM_DIM,
        binarize_gripper: bool = DEFAULT_BINARIZE_GRIPPER,
        gripper_binarize_threshold: float = DEFAULT_GRIPPER_BINARIZE_THRESHOLD,
        video_encoding: bool = False,
        video_backend: str | None = None,
        disable_stationary_frame_filtering: bool = False,
    ):
        valid_state_types = {"tquat", "xyzrpy", "joints"}
        valid_action_types = {"tquat", "delta_tquat", "xyzrpy", "delta_xyzrpy", "joints"}
        valid_types = {
            "tquat": 7,
            "delta_tquat": 7,
            "xyzrpy": 6,
            "delta_xyzrpy": 6,
            "joints": per_robot_arm_dim,
        }
        if returned_state_type not in valid_state_types or returned_action_type not in valid_action_types:
            raise ValueError(
                "returned_state_type and returned_action_type must be one of: "
                "tquat, delta_tquat, xyzrpy, delta_xyzrpy, joints"
            )
        self.root = Path(root)
        self.conn = duckdb.connect()
        self.dataset_paths = dataset_paths or list(DEFAULT_DATASET_PATHS)
        self.repo_id = repo_id
        self.robot_type = robot_type
        self.fps = fps
        self.robot_keys = robot_keys or list(DEFAULT_ROBOT_KEYS)
        self.source_action_is_joint = source_action_is_joint
        self.action_source_field = action_source_field
        self.returned_state_type = returned_state_type
        self.returned_action_type = returned_action_type
        self.delta_from_observation = delta_from_observation
        self.gripper_type = gripper_type
        self.cameras = cameras or list(DEFAULT_CAMERAS)
        self.image_batch_size = image_batch_size
        self.per_robot_arm_dim = per_robot_arm_dim
        self._arm_dims = valid_types
        self.state_dim = len(self.robot_keys) * (self._arm_dims[returned_state_type] + 1)
        self.action_dim = len(self.robot_keys) * (self._arm_dims[returned_action_type] + 1)
        self.binarize_gripper = binarize_gripper
        self.gripper_binarize_threshold = gripper_binarize_threshold
        self.disable_stationary_frame_filtering = disable_stationary_frame_filtering
        self.source_sql = self._build_source_sql(self.dataset_paths)
        self._source_column_types: dict[str, str] | None = None
        self._arm_action_is_joint_source: dict[str, bool] = {}
        self.video_encoding = video_encoding
        self.tcp_offset = rcs.GRIPPER_TCP_OFFSETS[self.gripper_type]
        self.ik = rcs.common.Pin(
            rcs.ROBOTS[robot_type].mjcf_model_path,
            rcs.ROBOTS[robot_type].attachment_site,
        )
        self.camera_resizers = {
            camera.name: v2.Resize(camera.resolution)
            for camera in self.cameras
        }

        self.lrds = self._create_output_dataset(video_backend=video_backend)

    def _create_output_dataset(self, video_backend: str | None = None):
        """Create the frame sink used by :meth:`parse_episode`.

        The conversion logic only relies on ``add_frame``, ``save_episode`` and
        ``finalize``.  Keeping construction behind this small hook lets other
        storage backends reuse the exact same filtering, IK and image handling
        without maintaining a copy of the converter.
        """
        return LeRobotDataset.create(
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
        action_names = []
        component_names = {
            "joints": ["joint_{}".format(i) for i in range(self.per_robot_arm_dim)],
            "tquat": ["x", "y", "z", "qx", "qy", "qz", "qw"],
            "delta_tquat": ["delta_x", "delta_y", "delta_z", "delta_qx", "delta_qy", "delta_qz", "delta_qw"],
            "xyzrpy": ["x", "y", "z", "roll", "pitch", "yaw"],
            "delta_xyzrpy": ["delta_x", "delta_y", "delta_z", "delta_roll", "delta_pitch", "delta_yaw"],
        }
        for robot_key in self.robot_keys:
            state_names.extend([f"{robot_key}_{name}" for name in component_names[self.returned_state_type]])
            state_names.append(f"{robot_key}_gripper")
            action_names.extend([f"{robot_key}_{name}" for name in component_names[self.returned_action_type]])
            action_names.append(f"{robot_key}_gripper")

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
            "shape": (self.action_dim,),
            "names": action_names,
        }
        return features

    def _build_source_sql(self, dataset_paths: list[str]) -> str:
        queries = []
        for path in dataset_paths:
            escaped = str(path).replace("'", "''")
            queries.append(f"SELECT * FROM read_parquet('{escaped}')")
        return " UNION ALL ".join(queries)

    def _get_source_column_types(self) -> dict[str, str]:
        if self._source_column_types is None:
            rows = self.conn.execute(f"DESCRIBE SELECT * FROM ({self.source_sql}) AS src").fetchall()
            self._source_column_types = {str(row[0]): str(row[1]) for row in rows}
        return self._source_column_types

    @staticmethod
    def _split_top_level_struct_fields(inner: str) -> list[str]:
        fields = []
        start = 0
        depth = 0
        in_quotes = False
        idx = 0
        while idx < len(inner):
            char = inner[idx]
            if char == '"':
                in_quotes = not in_quotes
            elif not in_quotes:
                if char == "(":
                    depth += 1
                elif char == ")":
                    depth -= 1
                elif char == "," and depth == 0:
                    fields.append(inner[start:idx].strip())
                    start = idx + 1
            idx += 1
        fields.append(inner[start:].strip())
        return [field for field in fields if field]

    @staticmethod
    def _parse_struct_field(field: str) -> tuple[str, str] | None:
        field = field.strip()
        if not field:
            return None
        if field.startswith('"'):
            end_quote = field.find('"', 1)
            if end_quote == -1:
                return None
            return field[1:end_quote], field[end_quote + 1 :].strip()
        name, sep, field_type = field.partition(" ")
        if not sep:
            return None
        return name, field_type.strip()

    @classmethod
    def _extract_struct_field_type(cls, struct_type: str, field_name: str) -> str | None:
        struct_type = struct_type.strip()
        if not struct_type.upper().startswith("STRUCT(") or not struct_type.endswith(")"):
            return None
        inner = struct_type[len("STRUCT(") : -1]
        for field in cls._split_top_level_struct_fields(inner):
            parsed = cls._parse_struct_field(field)
            if parsed is None:
                continue
            name, field_type = parsed
            if name == field_name:
                return field_type
        return None

    def _source_has_path(self, root: str, *fields: str) -> bool:
        field_type = self._get_source_column_types().get(root)
        for field in fields:
            if field_type is None:
                return False
            field_type = self._extract_struct_field_type(field_type, field)
        return field_type is not None

    def _arm_action_select(self, robot_key: str) -> str:
        for root in ("info", "env_action", "action"):
            if self._source_has_path(root, robot_key, self.action_source_field):
                self._arm_action_is_joint_source[robot_key] = self.source_action_is_joint
                alias = f"source_action_{robot_key}"
                return f"{root}.{robot_key}.{self.action_source_field} AS {alias}"

        msg = (
            f"Could not find action field '{self.action_source_field}' for robot '{robot_key}' "
            f"in info, env_action, or action."
        )
        raise ValueError(msg)

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
            [
                f"obs.{robot_key}.{field} AS observation_{field}_{robot_key}"
                for robot_key in self.robot_keys
                for field in ("joints", "tquat", "xyzrpy")
            ]
            + [f"obs.{robot_key}.gripper AS observation_gripper_{robot_key}" for robot_key in self.robot_keys]
        )
        action_selects = ",\n                    ".join(
            [self._arm_action_select(robot_key) for robot_key in self.robot_keys]
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
            arm_state = row[f"observation_{self.returned_state_type}_{robot_key}"]
            gripper_vec = np.asarray(gripper, dtype=np.float32)
            arm_state_vec = np.asarray(arm_state, dtype=np.float32)
            expected_state_shape = (self._arm_dims[self.returned_state_type],)
            if (
                joints_vec.shape != (self.per_robot_arm_dim,)
                or arm_state_vec.shape != expected_state_shape
                or gripper_vec.shape != (1,)
            ):
                msg = (
                    f"Unexpected observation shapes for robot '{robot_key}' at step {row['step']}: "
                    f"joints={joints_vec.shape}, {self.returned_state_type}={arm_state_vec.shape}, "
                    f"gripper={gripper_vec.shape}"
                )
                raise ValueError(msg)
            gripper_vec = self._maybe_binarize_gripper(gripper_vec)
            vectors.append(np.concatenate([arm_state_vec, gripper_vec]).astype(np.float32))

        return np.concatenate(vectors).astype(np.float32)

    def _convert_action_to_joint_space(
        self, row: pd.Series, next_row: pd.Series | None = None
    ) -> np.ndarray | None:
        actions = []
        is_delta_action = self.returned_action_type in {"delta_tquat", "delta_xyzrpy"}
        if is_delta_action and next_row is None:
            return None
        for robot_key in self.robot_keys:
            observation_joints = row[f"observation_joints_{robot_key}"]
            action_gripper = row[f"action_gripper_{robot_key}"]
            source_action = row[f"source_action_{robot_key}"]
            if (
                self._is_missing(observation_joints)
                or self._is_missing(source_action)
                or self._is_missing(action_gripper)
            ):
                msg = f"Missing action inputs for robot '{robot_key}' at step {row['step']}"
                raise ValueError(msg)

            observation_joints_vec = np.asarray(observation_joints, dtype=np.float64)
            source_action_vec = np.asarray(source_action, dtype=np.float64)
            action_gripper_vec = np.asarray(action_gripper, dtype=np.float32)
            if (
                observation_joints_vec.shape != (self.per_robot_arm_dim,)
                or source_action_vec.shape != ((self.per_robot_arm_dim,) if self._arm_action_is_joint_source.get(robot_key, self.source_action_is_joint) else (7,))
                or action_gripper_vec.shape != (1,)
            ):
                msg = (
                    f"Unexpected action shapes for robot '{robot_key}' at step {row['step']}: "
                    f"observation_joints={observation_joints_vec.shape}, "
                    f"source_action={source_action_vec.shape}, action_gripper={action_gripper_vec.shape}"
                )
                raise ValueError(msg)
            action_gripper_vec = self._maybe_binarize_gripper(action_gripper_vec)

            source_is_joint = self._arm_action_is_joint_source.get(robot_key, self.source_action_is_joint)
            if is_delta_action and self.delta_from_observation:
                next_observed_tquat = np.asarray(
                    next_row[f"observation_tquat_{robot_key}"], dtype=np.float64
                )
                observed_tquat = np.asarray(row[f"observation_tquat_{robot_key}"], dtype=np.float64)
                observed_pose = rcs.common.Pose(
                    translation=observed_tquat[:3], quaternion=observed_tquat[3:]
                )
                next_observed_pose = rcs.common.Pose(
                    translation=next_observed_tquat[:3], quaternion=next_observed_tquat[3:]
                )
                observed_xyzrpy = np.asarray(
                    row[f"observation_xyzrpy_unwrapped_{robot_key}"], dtype=np.float64
                )
                next_observed_xyzrpy = np.asarray(
                    next_row[f"observation_xyzrpy_unwrapped_{robot_key}"], dtype=np.float64
                )
                arm_action_vec = self._format_delta_pose(
                    observed_pose,
                    next_observed_pose,
                    observed_xyzrpy,
                    next_observed_xyzrpy,
                )
            elif source_is_joint and self.returned_action_type == "joints":
                arm_action_vec = source_action_vec.astype(np.float32)
            elif not source_is_joint and self.returned_action_type == "tquat":
                arm_action_vec = source_action_vec.astype(np.float32)
            elif source_is_joint:
                source_pose = self.ik.forward(source_action_vec, self.tcp_offset)
                if is_delta_action:
                    next_source = np.asarray(next_row[f"source_action_{robot_key}"], dtype=np.float64)
                    next_pose = self.ik.forward(next_source, self.tcp_offset)
                    arm_action_vec = self._format_delta_pose(
                        source_pose,
                        next_pose,
                        row.get(f"source_action_xyzrpy_{robot_key}"),
                        next_row.get(f"source_action_xyzrpy_{robot_key}"),
                    )
                else:
                    arm_action_vec = self._format_action_pose(source_pose)
            else:
                source_pose = rcs.common.Pose(translation=source_action_vec[:3], quaternion=source_action_vec[3:7])
                if self.returned_action_type in {"joints"}:
                    ik_joints: np.ndarray | None = self.ik.inverse(
                        source_pose, observation_joints_vec, tcp_offset=self.tcp_offset
                    )
                    if ik_joints is None:
                        msg = f"IK failed for robot '{robot_key}' at step {row['step']}, ignoring step"
                        warnings.warn(msg, stacklevel=1)
                        return None
                    arm_action_vec = np.asarray(ik_joints, dtype=np.float32)
                else:
                    if is_delta_action:
                        next_source = np.asarray(next_row[f"source_action_{robot_key}"], dtype=np.float64)
                        next_pose = rcs.common.Pose(
                            translation=next_source[:3], quaternion=next_source[3:7]
                        )
                        arm_action_vec = self._format_delta_pose(
                            source_pose,
                            next_pose,
                            row.get(f"source_action_xyzrpy_{robot_key}"),
                            next_row.get(f"source_action_xyzrpy_{robot_key}"),
                        )
                    else:
                        arm_action_vec = self._format_action_pose(source_pose)

            arm_action_vec = np.asarray(arm_action_vec, dtype=np.float32)

            actions.append(np.concatenate([arm_action_vec, action_gripper_vec]).astype(np.float32))

        concatenated = np.concatenate(actions).astype(np.float32)
        if concatenated.shape != (self.action_dim,):
            msg = f"Unexpected concatenated action shape {concatenated.shape} at step {row['step']}"
            raise ValueError(msg)
        return concatenated

    def _format_action_pose(self, target_pose: rcs.common.Pose) -> np.ndarray:
        if self.returned_action_type == "tquat":
            return np.concatenate([target_pose.translation(), target_pose.rotation_q()])
        if self.returned_action_type == "xyzrpy":
            return target_pose.xyzrpy()

        raise ValueError(f"Unsupported returned action type: {self.returned_action_type}")

    def _format_delta_pose(
        self,
        current_pose: rcs.common.Pose,
        next_pose: rcs.common.Pose,
        current_xyzrpy: np.ndarray | None = None,
        next_xyzrpy: np.ndarray | None = None,
    ) -> np.ndarray:
        delta_pose = next_pose * current_pose.inverse()
        # q and -q represent the same rotation. Select the shortest-arc
        # representative before converting to RPY; otherwise a sign flip can
        # appear as an artificial +/-pi Euler rotation.
        delta_quaternion = np.asarray(delta_pose.rotation_q(), dtype=np.float64)
        if delta_quaternion[3] < 0:
            delta_quaternion = -delta_quaternion
        if self.returned_action_type == "delta_tquat":
            return np.concatenate([delta_pose.translation(), delta_quaternion])
        if self.returned_action_type == "delta_xyzrpy":
            current_xyzrpy = current_xyzrpy if current_xyzrpy is not None else current_pose.xyzrpy()
            next_xyzrpy = next_xyzrpy if next_xyzrpy is not None else next_pose.xyzrpy()
            # Compute the local rotation from the relative pose instead of
            # subtracting two independently chosen Euler-angle branches. The
            # latter can turn a small motion into a +/-pi jump near an Euler
            # representation boundary.
            # RCS's native rotvec is the shortest rotational representation of
            # the relative pose and is not subject to Euler branch selection.
            rotation_delta = np.asarray(delta_pose.rotvec(), dtype=np.float64)[3:]
            if np.linalg.norm(rotation_delta) > np.pi + 1e-5:
                raise ValueError(
                    f"Relative rotation exceeds the shortest-arc bound at delta conversion: "
                    f"{rotation_delta}"
                )
            return np.concatenate(
                [
                    next_xyzrpy[:3] - current_xyzrpy[:3],
                    rotation_delta,
                ]
            )
        raise ValueError(f"Unsupported returned action type: {self.returned_action_type}")

    @staticmethod
    def _rotation_vector_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
        """Return the shortest local rotational increment from an xyzw quaternion."""
        quaternion = np.asarray(quaternion, dtype=np.float64)
        quaternion /= np.linalg.norm(quaternion)
        if quaternion[3] < 0:
            quaternion = -quaternion
        vector = quaternion[:3]
        vector_norm = np.linalg.norm(vector)
        if vector_norm < 1e-12:
            return 2.0 * vector
        angle = 2.0 * np.arctan2(vector_norm, quaternion[3])
        return vector * (angle / vector_norm)

    @staticmethod
    def _unwrap_rpy_sequence(rpy_values: np.ndarray) -> np.ndarray:
        """Keep equivalent Euler branches continuous across an episode."""
        continuous = np.empty_like(rpy_values)
        continuous[0] = rpy_values[0]
        two_pi = 2 * np.pi

        for index in range(1, len(rpy_values)):
            rpy = rpy_values[index]
            previous = continuous[index - 1]
            candidates = []
            alternate = rpy.copy()
            alternate[3:] = np.array([rpy[3] + np.pi, np.pi - rpy[4], rpy[5] + np.pi])
            for candidate in (rpy, alternate):
                turns = np.round((previous[3:] - candidate[3:]) / two_pi)
                adjusted = candidate.copy()
                adjusted[3:] += turns * two_pi
                candidates.append(adjusted)
            continuous[index] = min(
                candidates, key=lambda candidate: np.linalg.norm(candidate[3:] - previous[3:])
            )

        return continuous

    def _prepare_transition_table(self, table: pd.DataFrame) -> pd.DataFrame:
        if len(table) == 0:
            return table

        df = table.copy()  # noqa: PD901
        df["observation_state"] = df.apply(self._build_observation_state, axis=1)
        returned_action_type = getattr(self, "returned_action_type", DEFAULT_RETURNED_ACTION_TYPE)
        delta_from_observation = getattr(self, "delta_from_observation", DEFAULT_DELTA_FROM_OBSERVATION)
        if returned_action_type == "delta_xyzrpy" and not delta_from_observation:
            for robot_key in self.robot_keys:
                action_rpy = []
                for _, row in df.iterrows():
                    source_action = row[f"source_action_{robot_key}"]
                    source_is_joint = self._arm_action_is_joint_source.get(
                        robot_key, self.source_action_is_joint
                    )
                    source_vec = np.asarray(source_action, dtype=np.float64)
                    source_pose = (
                        self.ik.forward(source_vec, self.tcp_offset)
                        if source_is_joint
                        else rcs.common.Pose(translation=source_vec[:3], quaternion=source_vec[3:7])
                    )
                    action_rpy.append(source_pose.xyzrpy())
                unwrapped = self._unwrap_rpy_sequence(np.stack(action_rpy))
                df[f"source_action_xyzrpy_{robot_key}"] = list(unwrapped)
        if returned_action_type == "delta_xyzrpy" and delta_from_observation:
            for robot_key in self.robot_keys:
                observation_xyzrpy = np.stack(
                    [
                        np.asarray(row[f"observation_xyzrpy_{robot_key}"], dtype=np.float64)
                        for _, row in df.iterrows()
                    ]
                )
                unwrapped = self._unwrap_rpy_sequence(observation_xyzrpy)
                df[f"observation_xyzrpy_unwrapped_{robot_key}"] = list(unwrapped)
        if returned_action_type in {"delta_tquat", "delta_xyzrpy"} and not delta_from_observation:
            df["action_vector"] = [
                self._convert_action_to_joint_space(row, next_row)
                for (_, row), (_, next_row) in zip(df.iloc[:-1].iterrows(), df.iloc[1:].iterrows())
            ] + [None]
        elif returned_action_type in {"delta_tquat", "delta_xyzrpy"}:
            df["action_vector"] = [
                self._convert_action_to_joint_space(row, next_row)
                for (_, row), (_, next_row) in zip(df.iloc[:-1].iterrows(), df.iloc[1:].iterrows())
            ] + [None]
        else:
            df["action_vector"] = df.apply(self._convert_action_to_joint_space, axis=1)

        df = df[df["action_vector"].notna()]  # noqa: PD901
        if self.disable_stationary_frame_filtering:
            return df.reset_index(drop=True)

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
            torch.frombuffer(bytearray(image_bytes), dtype=torch.uint8)
            for image_bytes in image_bytes_list
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
    source_action_is_joint: bool = DEFAULT_SOURCE_ACTION_IS_JOINT,
    action_source_field: str = DEFAULT_ACTION_SOURCE_FIELD,
    returned_state_type: str = DEFAULT_RETURNED_STATE_TYPE,
    returned_action_type: str = DEFAULT_RETURNED_ACTION_TYPE,
    delta_from_observation: bool = DEFAULT_DELTA_FROM_OBSERVATION,
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
    disable_stationary_frame_filtering: bool = False,
) -> None:
    import torch  # noqa: F401
    from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: F401
    from torchvision.io import decode_jpeg  # noqa: F401
    from torchvision.transforms import v2  # noqa: F401

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
        source_action_is_joint=source_action_is_joint,
        action_source_field=action_source_field,
        returned_state_type=returned_state_type,
        returned_action_type=returned_action_type,
        delta_from_observation=delta_from_observation,
        cameras=cameras,
        image_batch_size=image_batch_size,
        per_robot_arm_dim=per_robot_arm_dim,
        binarize_gripper=binarize_gripper,
        gripper_binarize_threshold=gripper_binarize_threshold,
        disable_stationary_frame_filtering=disable_stationary_frame_filtering,
        video_encoding=video_encoding,
        video_backend=video_backend,
    )
    converter.generate_examples(success=success, n=n)


if __name__ == "__main__":
    run_conversion()
