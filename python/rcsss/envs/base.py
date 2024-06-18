"""Gym API."""

from enum import Enum
from typing import Any, Literal, TypeAlias, TypedDict, cast

import gymnasium as gym
import numpy as np
from rcsss import common

Vec7Type: TypeAlias = np.ndarray[Literal[7], np.dtype[np.float64]]
Vec3Type: TypeAlias = np.ndarray[Literal[3], np.dtype[np.float64]]

RPY_SPACE = gym.spaces.Box(low=np.deg2rad(-180), high=np.deg2rad(180), shape=(3,))
XYZ_SPACE = gym.spaces.Box(low=np.array([-0.855, -0.855, 0]), high=np.array([0.855, 0.855, 0.1188]), shape=(3,))

POSE_SPACE = gym.spaces.Dict({"rpy": RPY_SPACE, "xyz": XYZ_SPACE})


class PoseDictType(TypedDict):
    rpy: Vec3Type
    xyz: Vec3Type


ANGLES_SPACE = gym.spaces.Box(
    low=np.array([-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895]),
    high=np.array([2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895]),
    dtype=np.float32,
    shape=(7,),
)

ARM_OBS_SPACE = gym.spaces.Dict(
    {
        "pose": POSE_SPACE,
        "angles": ANGLES_SPACE,
    }
)


class ArmObs(TypedDict):
    pose: PoseDictType
    angles: Vec7Type


# frame
COLOR_SPACE = IR_SPACE = DEPTH_SPACE = gym.spaces.Box(low=0, high=255, dtype=np.uint8)
ACC_SPACE = GYRO_SPACE = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)

# TODO: move some of this info to "info"
FRAME_SPACE = gym.spaces.Dict(
    {
        "camera": gym.spaces.Dict(
            {
                "color": COLOR_SPACE,
                "ir": IR_SPACE,
                "depth": DEPTH_SPACE,
            }
        ),
        "imu": gym.spaces.Dict(
            {
                "acc_sample": ACC_SPACE,
                "gyro_sample": GYRO_SPACE,
            }
        ),
    }
)


class CameraDictType(TypedDict):
    color: np.ndarray
    ir: np.ndarray | None
    depth: np.ndarray | None


class ImuDictType(TypedDict):
    acc_sample: np.ndarray
    gyro_sample: np.ndarray


class FrameDictType(TypedDict):
    camera: CameraDictType
    imu: ImuDictType | None


class ControlMode(Enum):
    ANGLES = 1
    CARTESIAN = 2


CartOrAngleControl: TypeAlias = Vec7Type | PoseDictType


class FR3Env(gym.Env[ArmObs, CartOrAngleControl]):
    """Joint Gym Environment for Franka Research 3."""

    def __init__(self, robot: common.Robot, control_mode: ControlMode):
        self.robot = robot
        self.control_mode = control_mode
        self.action_space: gym.spaces.Space
        self.observation_space: gym.spaces.Space
        if control_mode == ControlMode.ANGLES:
            self.action_space = ANGLES_SPACE
        else:
            self.action_space = POSE_SPACE
        self.observation_space = ARM_OBS_SPACE

    def _get_obs(self) -> ArmObs:
        pose = self.robot.get_cartesian_position()
        rpy = pose.rotation_rpy()
        xyz = pose.translation()
        return ArmObs(
            pose=PoseDictType(rpy=np.array([rpy.roll, rpy.pitch, rpy.yaw]), xyz=np.array(xyz)),
            angles=self.robot.get_joint_position(),
        )

    def angle_step(self, act: Vec7Type) -> None:
        self.robot.set_joint_position(act)

    def cartesian_step(self, act: common.Pose) -> None:
        self.robot.set_cartesian_position(act)

    def step(self, act: CartOrAngleControl) -> tuple[ArmObs, float, bool, bool, dict]:
        if self.control_mode == ControlMode.ANGLES and isinstance(act, np.ndarray):
            self.angle_step(act)
        elif self.control_mode == ControlMode.CARTESIAN and isinstance(act, dict):
            act = cast(PoseDictType, act)
            self.cartesian_step(common.Pose(act["rpy"], act["xyz"]))
        else:
            msg = "Given type is not matching control mode!"
            raise RuntimeError(msg)
        return self._get_obs(), 0, False, False, {}

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[ArmObs, dict[str, Any]]:
        if seed is not None:
            msg = "seeding not implemented yet"
            raise NotImplementedError(msg)
        if options is not None:
            msg = "options not implemented yet"
            raise NotImplementedError(msg)
        return self._get_obs(), {}
