"""Robot control stack python bindings."""

import os
from dataclasses import dataclass
from typing import Any

import numpy as np
from rcs._core import __version__, common

from rcs import camera, envs, hand, sim


def _rcs_prefix() -> str:
    env_prefix = os.environ.get("RCS_PREFIX")
    if env_prefix:
        return os.path.abspath(env_prefix)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))


RCS_PREFIX = _rcs_prefix()


# TODO: assets must be "downloaded" first time this is imported
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
}


GRIPPER_PATHS: dict[common.GripperType, str] = {
    common.GripperType.FrankaHand: "assets/grippers/franka_hand/franka_hand.xml",
    common.GripperType("Robotiq2F85"): "assets/grippers/robotiq_2f85/robotiq_2f85.xml",
}

SCENE_PATHS: dict[str, str] = {"empty_world": "assets/scenes/empty_world/scene.xml"}

OBJECT_PATHS: dict[str, str] = {
    "fr3_duo_mount": "assets/objects/fr3_duo_mount/fr3_duo_mount.xml",
    "robotiq_d405_mount": "assets/objects/robotiq_d405_mount/robotiq_d405_mount.xml",
    "green_cube": "assets/objects/green_cube/green_cube.xml",
}

CAMERA_PATHS: dict[str, str] = {
    "d405": "assets/cameras/d405/d405.xml",
    "zed_mini": "assets/cameras/zed_mini/zed_mini.xml",
}

# we add our task classes here
TASKS: dict[str, Any] = {}

DEFAULT_TRANSFORMS = {
    "FR3_ROBOTIQ_GRIPPER": common.Pose(
        translation=np.array([0.0, 0.0, 0.0]), quaternion=np.array([0.0, 0.0, 0.7071068, 0.7071068])
    ),
    "FR3_ROBOTIQ_WRIST_D405_MOUNT": common.Pose(
        translation=np.array([0.0, 0.0, 0.0]), quaternion=np.array([0.0, 0.0, 0.7071068, 0.7071068])
    ),
    "FR3_ROBOTIQ_WRIST_D405_CAMERA": common.Pose(
        translation=np.array([0.060, 0.0, 0.0665]), rpy_vector=np.array([-np.pi / 2, -np.pi * 11 / 18, 0.0])
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
    "TASKS",
]
