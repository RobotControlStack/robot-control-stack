"""Robot control stack python bindings."""

import io
import os
import zipfile
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

import numpy as np
import requests
from rcs._core import __version__, common

from rcs import camera, envs, hand, sim

GITHUB_ASSET_ARCHIVE_URL = "https://github.com/RobotControlStack/robot-control-stack/archive/refs/tags/{tag}.zip"
REQUIRED_ASSET = Path("assets/scenes/empty_world/scene.xml")


def download_assets(
    version: str,
    prefix: Path,
    github_asset_archive_url: str,
    assets_dirname: str = "assets",
) -> None:
    tag = f"v{version.split('+', maxsplit=1)[0]}"
    response = requests.get(github_asset_archive_url.format(tag=tag), timeout=60)
    response.raise_for_status()

    found_assets = False
    with zipfile.ZipFile(io.BytesIO(response.content)) as archive:
        for member in archive.infolist():
            parts = PurePosixPath(member.filename).parts
            if len(parts) < 2 or parts[1] != assets_dirname:
                continue

            found_assets = True
            target = prefix.joinpath(*parts[1:])
            if member.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(archive.read(member))

    if not found_assets:
        msg = f"Could not find {assets_dirname} in the downloaded GitHub archive."
        raise FileNotFoundError(msg)


def get_prefix(
    env_prefix: str | None,
    editable_prefix: Path,
    default_prefix: Path,
    version: str,
    github_asset_archive_url: str,
    assets_dirname: str = "assets",
    download_fn: Callable[[str, Path, str, str], None] = download_assets,
) -> str:
    def has_required_assets(prefix: Path) -> bool:
        return (prefix / REQUIRED_ASSET).is_file()

    if env_prefix:
        prefix = Path(env_prefix).expanduser().resolve()
    else:
        editable_prefix = editable_prefix.resolve()
        if (editable_prefix / assets_dirname).is_dir():
            return str(editable_prefix)
        prefix = default_prefix.resolve()

    assets_dir = prefix / assets_dirname
    if not assets_dir.is_dir() or not has_required_assets(prefix):
        prefix.mkdir(parents=True, exist_ok=True)
        print(f"Assets not found at {assets_dir}, downloading them now.")
        download_fn(version, prefix, github_asset_archive_url, assets_dirname)

    return str(prefix)


RCS_PREFIX = get_prefix(
    os.environ.get("RCS_PREFIX"),
    Path(__file__).resolve().parents[2],
    Path.home() / ".rcs",
    __version__,
    GITHUB_ASSET_ARCHIVE_URL,
)


@dataclass(kw_only=True)
class RobotMetaConfig:

    mjcf_model_path: str
    """Path to the Mujoco XML file that describes the kinematics."""
    # robot_type: common.RobotType
    # """Type of the robot. Checkout all registered types by RobotType.get_all()"""
    dof: int
    "Number of degree of freedom of the robot arm"
    q_home: np.ndarray
    """joint angles in radiant of the robots task independent default home pose, shape: (N,)"""
    joint_limits: np.ndarray
    """hard joint limits of this robot, shape (2, N)"""
    attachment_site: str = "attachment_site"
    """mjcf site to use for IK"""


ROBOTS: dict[common.RobotType, RobotMetaConfig] = {
    common.RobotType.FR3: RobotMetaConfig(
        mjcf_model_path="assets/robots/fr3/fr3.xml",
        dof=7,
        q_home=np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, 0.0]),
        joint_limits=np.array(
            [
                [-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895],
                [2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895],
            ]
        ),
    ),
    common.RobotType.Panda: RobotMetaConfig(
        mjcf_model_path="assets/robots/panda/panda.xml",
        dof=7,
        q_home=np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, 0.0]),
        joint_limits=np.array(
            [
                [
                    -166.0 / 180.0 * np.pi,
                    -101.0 / 180.0 * np.pi,
                    -166.0 / 180.0 * np.pi,
                    -176.0 / 180.0 * np.pi,
                    -166.0 / 180.0 * np.pi,
                    -1.0 / 180.0 * np.pi,
                    -166.0 / 180.0 * np.pi,
                ],
                [
                    166.0 / 180.0 * np.pi,
                    101.0 / 180.0 * np.pi,
                    166.0 / 180.0 * np.pi,
                    -4.0 / 180.0 * np.pi,
                    166.0 / 180.0 * np.pi,
                    215.0 / 180.0 * np.pi,
                    166.0 / 180.0 * np.pi,
                ],
            ]
        ),
    ),
    common.RobotType("XArm7"): RobotMetaConfig(
        mjcf_model_path="assets/robots/xarm7/xarm7.xml",
        dof=7,
        q_home=np.array([0, -45.0 / 180.0 * np.pi, 0, 15.0 / 180.0 * np.pi, 0, -25.0 / 180.0 * np.pi, 0]),
        joint_limits=np.array(
            [
                [-2 * np.pi, -2.094395, -2 * np.pi, -3.92699, -2 * np.pi, -np.pi, -2 * np.pi],
                [2 * np.pi, 2.059488, 2 * np.pi, 0.191986, 2 * np.pi, 1.692969, 2 * np.pi],
            ]
        ),
    ),
    common.RobotType("UR5e"): RobotMetaConfig(
        mjcf_model_path="assets/robots/ur5e/ur5e.xml",
        dof=6,
        q_home=np.array([0.0, -2.02711196, 1.64630026, -1.18999615, -1.57079762, 0.0]),
        joint_limits=np.array(
            [
                [-2 * np.pi, -2 * np.pi, -1 * np.pi, -2 * np.pi, -2 * np.pi, -2 * np.pi],
                [2 * np.pi, 2 * np.pi, 1 * np.pi, 2 * np.pi, 2 * np.pi, 2 * np.pi],
            ]
        ),
    ),
    common.RobotType("SO101"): RobotMetaConfig(
        mjcf_model_path="assets/robots/so101/so101.xml",
        dof=5,
        q_home=np.array([-0.01914898, -1.90521916, 1.56476701, 1.04783839, -1.40323926]),
        joint_limits=np.array(
            [
                [
                    -1.9198621771937616,
                    -1.9198621771937634,
                    -1.7453292519943295,
                    -1.6580627969561903,
                    -2.7925268969992407,
                ],
                [1.9198621771937616, 1.9198621771937634, 1.5707963267948966, 1.6580627969561903, 2.7925268969992407],
            ]
        ),
        attachment_site="gripper",
    ),
    common.RobotType("Yam"): RobotMetaConfig(
        mjcf_model_path="assets/robots/yam/yam.xml",
        dof=6,
        q_home=np.array([0.0, 1.047, 1.047, 0.0, 0.0, 0.0]),
        joint_limits=np.array(
            [
                [-2.61799, 0.0, 0.0, -1.69297, -1.5708, -2.0944],
                [3.14159, 3.66519, 3.14159, 1.5708, 1.5708, 2.0944],
            ]
        ),
        attachment_site="tcp_site",
    ),
}


GRIPPER_PATHS: dict[common.GripperType, str] = {
    common.GripperType.FrankaHand: "assets/grippers/franka_hand/franka_hand.xml",
    common.GripperType("Robotiq2F85"): "assets/grippers/robotiq_2f85/robotiq_2f85.xml",
}

GRIPPER_TCP_OFFSETS: dict[common.GripperType, common.Pose] = {
    common.GripperType.FrankaHand: common.Pose(pose_matrix=common.FrankaHandTCPOffset()),
    common.GripperType("Robotiq2F85"): common.Pose(translation=np.array([0, 0.0, 0.1493])),
    # The yam gripper is part of the robot mjcf, hence it needs no entry in GRIPPER_PATHS
    # and no mount offset, only the offset from the flange to the point between the fingers.
    common.GripperType("Yam"): common.Pose(translation=np.array([0.0, 0.0, 0.1347])),
}

GRIPPER_MOUNT_OFFSETS: dict[common.GripperType, common.Pose] = {
    common.GripperType.FrankaHand: common.Pose(
        rotation=common.FrankaHandTCPOffset()[:3, :3], translation=np.array([0.0, 0.0, 0.0])
    ),
    common.GripperType("Robotiq2F85"): common.Pose(
        translation=np.array([0.0, 0.0, 0.0]), quaternion=np.array([0.0, 0.0, 0.7071068, 0.7071068])
    ),
}

SCENE_PATHS: dict[str, str] = {"empty_world": "assets/scenes/empty_world/scene.xml"}

OBJECT_PATHS: dict[str, str] = {
    "fr3_duo_mount": "assets/objects/fr3_duo_mount/fr3_duo_mount.xml",
    "fr3_single_mount": "assets/objects/fr3_single_mount/fr3_single_mount.xml",
    "robotiq_d405_mount": "assets/objects/robotiq_d405_mount/robotiq_d405_mount.xml",
    "droid_wrist_mount": "assets/objects/droid_wrist_mount/droid_wrist_mount.xml",
    "green_cuboid": "assets/objects/green_cuboid/green_cuboid.xml",
    "green_cube": "assets/objects/green_cube/green_cube.xml",
    "red_cube": "assets/objects/red_cube/red_cube.xml",
}

CAMERA_PATHS: dict[str, str] = {
    "d405": "assets/cameras/d405/d405.xml",
    "zed_mini": "assets/cameras/zed_mini/zed_mini.xml",
    "zed2i": "assets/cameras/zed2i/zed2i.xml",
}

# we add our task classes here
TASKS: dict[str, Any] = {}

DEFAULT_TRANSFORMS = {
    "FR3_ROBOTIQ_WRIST_D405_MOUNT": common.Pose(
        translation=np.array([0.0, 0.0, 0.0]), quaternion=np.array([0.0, 0.0, 0.7071068, 0.7071068])
    ),
    "FR3_ROBOTIQ_WRIST_D405_CAMERA": common.Pose(
        translation=np.array([0.060, 0.0, 0.0665]), rpy_vector=np.array([np.pi, -np.pi * 11 / 18, 0])
    ),
    "FR3_DUOMOUNT_HEIGHT_OFFSET": common.Pose(
        translation=np.array([0.0, 0.0, 0.342]), quaternion=np.array([0.0, 0.0, 0.0, 1.0])
    ),
    "FR3_DUOMOUNT_BASE": common.Pose(translation=np.array([0.0, 0.0, 0.0]), quaternion=np.array([0.0, 0.0, 0.0, 1.0])),
    "FR3_DUOMOUNT_LEFT_ROBOT": common.Pose(
        translation=np.array([0.0, 0.05018, 0.0]), quaternion=np.array([-0.436978, 0.0225312, -0.243326, 0.865641])
    ),
    "FR3_DUOMOUNT_RIGHT_ROBOT": common.Pose(
        translation=np.array([0.0, -0.05018, 0.0]), quaternion=np.array([0.436978, 0.0225312, 0.243326, 0.865641])
    ),
    "FR3_DUOMOUNT_ZEDMINI_CAMERA": common.Pose(
        translation=np.array([0.0113, -0.0245, 0.695]),
        rpy_vector=np.array([0.0, np.pi * 41 / 180, 0.0]),
    ),
}

HOME_POSITIONS = {
    # calculated from
    # "left": {"xyzrpy": [0.61, 0.21, 0.40, -np.pi, np.deg2rad(-20), 0], "gripper": [0]},
    # "right": {"xyzrpy": [0.61, -0.21, 0.40, -np.pi, np.deg2rad(-20), 0], "gripper": [0]},
    "FR3_DUO_LEFT": np.array([0.48797692, -0.57224476, -0.58536988, -2.57958827, 0.86400183, 2.0530809, -0.85965005]),
    "FR3_DUO_RIGHT": np.array([-0.48797676, -0.57224472, 0.58536959, -2.57958788, -0.86400148, 2.05308196, 0.85965057]),
    "FR3_DROID": np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, 0.0]),
}

# Append RCS package prefix to all asset paths
for path_dict in (SCENE_PATHS, OBJECT_PATHS, CAMERA_PATHS):
    for name, path in path_dict.items():
        abs_path = os.path.join(RCS_PREFIX, path)
        if not os.path.isfile(abs_path):
            error_msg = f"Asset {name} not found at path: {abs_path}. Please make sure to download the assets."
            raise FileNotFoundError(error_msg)
        else:
            path_dict[name] = abs_path
for gripper_type, path in GRIPPER_PATHS.items():
    abs_path = os.path.join(RCS_PREFIX, path)
    if not os.path.isfile(abs_path):
        error_msg = f"Asset {gripper_type} not found at path: {abs_path}. Please make sure to download the assets."
        raise FileNotFoundError(error_msg)
    GRIPPER_PATHS[gripper_type] = abs_path
for robot_name, robot_cfg in ROBOTS.items():
    abs_path = os.path.join(RCS_PREFIX, robot_cfg.mjcf_model_path)
    if not os.path.isfile(abs_path):
        error_msg = f"Robot model {robot_name} not found at path: {abs_path}. Please make sure to download the assets."
        raise FileNotFoundError(error_msg)
    else:
        robot_cfg.mjcf_model_path = abs_path

# make submodules available
__all__ = [
    "__doc__",
    "__version__",
    "common",
    "sim",
    "camera",
    "envs",
    "hand",
    "ROBOTS",
    "GRIPPER_PATHS",
    "SCENE_PATHS",
    "OBJECT_PATHS",
    "CAMERA_PATHS",
    "HOME_POSITIONS",
    "TASKS",
]
