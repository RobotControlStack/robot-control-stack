"""Gym API."""

import copy
import logging
from enum import Enum, auto
from typing import Annotated, Any, Literal, TypeAlias, cast

import gymnasium as gym
import numpy as np
from rcs.camera.interface import BaseCameraSet
from rcs.envs.space_utils import (
    ActObsInfoWrapper,
    RCSpaceType,
    Vec6Type,
    Vec7Type,
    Vec18Type,
    VecType,
    get_space,
    get_space_keys,
)
from rcs.hand.interface import BaseHand

from rcs import common

_logger = logging.getLogger(__name__)


class TRPYDictType(RCSpaceType):
    """Pose format is in transpose[3],r,p,y"""

    xyzrpy: Annotated[
        Vec6Type,
        gym.spaces.Box(
            low=np.array([-0.855, -0.855, 0, -np.deg2rad(180), -np.deg2rad(180), -np.deg2rad(180)]),
            high=np.array([0.855, 0.855, 1.188, np.deg2rad(180), np.deg2rad(180), np.deg2rad(180)]),
            dtype=np.float64,
        ),
    ]


class LimitedTRPYRelDictType(RCSpaceType):
    xyzrpy: Annotated[
        Vec6Type,
        lambda max_cart_mov, max_angle_mov: gym.spaces.Box(
            low=np.array(3 * [-max_cart_mov] + 3 * [-max_angle_mov]),
            high=np.array(3 * [max_cart_mov] + 3 * [max_angle_mov]),
            dtype=np.float64,
        ),
        "cart_limits",
    ]


class TQuatDictType(RCSpaceType):
    tquat: Annotated[
        Vec7Type,
        gym.spaces.Box(
            low=np.array([-0.855, -0.855, 0] + [-1] + [-np.inf] * 3),
            high=np.array([0.855, 0.855, 1.188] + [1] + [np.inf] * 3),
            dtype=np.float64,
        ),
    ]


class LimitedTQuatRelDictType(RCSpaceType):
    tquat: Annotated[
        Vec7Type,
        lambda max_cart_mov: gym.spaces.Box(
            low=np.array(3 * [-max_cart_mov] + [-1] + [-np.inf] * 3),
            high=np.array(3 * [max_cart_mov] + [1] + [np.inf] * 3),
            dtype=np.float64,
        ),
        "cart_limits",
    ]


class JointsDictType(RCSpaceType):
    joints: Annotated[
        VecType,
        lambda low, high: gym.spaces.Box(
            low=np.array(low),
            high=np.array(high),
            dtype=np.float64,
        ),
        "joint_limits",
    ]


class LimitedJointsRelDictType(RCSpaceType):
    joints: Annotated[
        VecType,
        lambda max_joint_mov, dof=7: gym.spaces.Box(
            low=np.array(dof * [-max_joint_mov]),
            high=np.array(dof * [max_joint_mov]),
            dtype=np.float64,
        ),
        "joint_limits",
    ]


class GripperDictType(RCSpaceType):
    # 0 for closed, 1 for open (>=0.5 for open)
    gripper: Annotated[float, gym.spaces.Box(low=0, high=1, dtype=np.float32)]


class HandBinDictType(RCSpaceType):
    # 0 for closed, 1 for open (>=0.5 for open)
    hand: Annotated[float, gym.spaces.Box(low=0, high=1, dtype=np.float32)]


class HandVecDictType(RCSpaceType):
    hand: Annotated[
        Vec18Type,
        gym.spaces.Box(
            low=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32,
        ),
    ]


class CameraDataDictType(RCSpaceType):
    data: Annotated[
        np.ndarray,
        # needs to be filled with values downstream
        lambda height, width, color_dim=3, dtype=np.uint8, low=0, high=255: gym.spaces.Box(
            low=low,
            high=high,
            shape=(height, width, color_dim),
            dtype=dtype,
        ),
        "frame",
    ]
    intrinsics: Annotated[
        np.ndarray[tuple[Literal[3], Literal[4]], np.dtype[np.float64]] | None,
        gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(3, 4),
            dtype=np.float64,
        ),
    ]
    extrinsics: Annotated[
        np.ndarray[tuple[Literal[4], Literal[4]], np.dtype[np.float64]] | None,
        gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(4, 4),
            dtype=np.float64,
        ),
    ]


class CameraDictType(RCSpaceType):
    frames: dict[
        Annotated[str, "camera_names"],
        dict[
            Annotated[str, "camera_type"],  # "rgb" or "depth"
            CameraDataDictType,
        ],
    ]


# joining works with inheritance but need to inherit from protocol again
class ArmObsType(TQuatDictType, JointsDictType, TRPYDictType): ...


CartOrJointContType: TypeAlias = TQuatDictType | JointsDictType | TRPYDictType
LimitedCartOrJointContType: TypeAlias = LimitedTQuatRelDictType | LimitedJointsRelDictType | LimitedTRPYRelDictType


class ControlMode(Enum):
    JOINTS = auto()
    CARTESIAN_TRPY = auto()
    CARTESIAN_TQuat = auto()


def get_dof(robot: common.Robot) -> int:
    """Returns the number of degrees of freedom of the robot."""
    return common.robots_meta_config(robot.get_parameters().robot_type).dof


def get_joint_limits(robot: common.Robot) -> tuple[np.ndarray, np.ndarray]:
    """Returns the joint limits of the robot.

    The first element is the lower limit, the second element is the upper limit.
    """
    limits = common.robots_meta_config(robot.get_parameters().robot_type).joint_limits
    return limits[0], limits[1]


def get_home_position(robot: common.Robot) -> np.ndarray:
    """Returns the home position of the robot."""
    return common.robots_meta_config(robot.get_parameters().robot_type).q_home


class RobotEnv(gym.Env):
    """Joint Gym Environment for a single robot arm.

    Top view of on the robot. Robot faces into x direction.
    z direction faces upwards. (Right handed coordinate axis)
        ^ x
        |
    <-- RobotBase
    y
    """

    def __init__(self, robot: common.Robot, control_mode: ControlMode, home_on_reset: bool = False):
        self.robot = robot
        self._control_mode_overrides = [control_mode]
        self.action_space: gym.spaces.Dict
        self.observation_space: gym.spaces.Dict
        self.home_on_reset = home_on_reset
        low, high = get_joint_limits(self.robot)
        if control_mode == ControlMode.JOINTS:
            self.action_space = get_space(JointsDictType, params={"joint_limits": {"low": low, "high": high}})
        elif control_mode == ControlMode.CARTESIAN_TRPY:
            self.action_space = get_space(TRPYDictType)
        elif control_mode == ControlMode.CARTESIAN_TQuat:
            self.action_space = get_space(TQuatDictType)
        else:
            msg = "Control mode not recognized!"
            raise ValueError(msg)
        self.observation_space = get_space(ArmObsType, params={"joint_limits": {"low": low, "high": high}})
        self.joints_key = get_space_keys(JointsDictType)[0]
        self.trpy_key = get_space_keys(TRPYDictType)[0]
        self.tquat_key = get_space_keys(TQuatDictType)[0]
        self.prev_action: dict | None = None

    def get_unwrapped_control_mode(self, idx: int) -> ControlMode:
        """Returns the unwrapped control mode at a certain index. 0 is the base control mode, -1 the last."""
        return self._control_mode_overrides[idx]

    def get_base_control_mode(self) -> ControlMode:
        """Returns the unwrapped control mode"""
        return self._control_mode_overrides[0]

    def get_control_mode(self) -> ControlMode:
        """Use this function to get the current wrapped control mode"""
        return self._control_mode_overrides[-1]

    def override_control_mode(self, control_mode: ControlMode):
        """Sets a new wrapped control mode.
        Use this in a wrapper that wants to modify the control mode"""
        self._control_mode_overrides.append(control_mode)

    def get_obs(self) -> ArmObsType:
        return ArmObsType(
            tquat=np.concatenate(
                [self.robot.get_cartesian_position().translation(), self.robot.get_cartesian_position().rotation_q()]
            ),
            joints=self.robot.get_joint_position(),
            xyzrpy=self.robot.get_cartesian_position().xyzrpy(),
        )

    def step(self, action: CartOrJointContType) -> tuple[ArmObsType, float, bool, bool, dict]:
        action_dict = cast(dict, action)
        if (
            self.get_base_control_mode() == ControlMode.CARTESIAN_TQuat
            and self.tquat_key not in action_dict
            or self.get_base_control_mode() == ControlMode.CARTESIAN_TRPY
            and self.trpy_key not in action_dict
            or self.get_base_control_mode() == ControlMode.JOINTS
            and self.joints_key not in action_dict
        ):
            msg = "Given type is not matching control mode!"
            raise RuntimeError(msg)

        if self.get_base_control_mode() == ControlMode.JOINTS and (
            self.prev_action is None
            or not np.allclose(action_dict[self.joints_key], self.prev_action[self.joints_key], atol=1e-03, rtol=0)
        ):
            self.robot.set_joint_position(action_dict[self.joints_key])
        elif self.get_base_control_mode() == ControlMode.CARTESIAN_TRPY and (
            self.prev_action is None
            or not np.allclose(action_dict[self.trpy_key], self.prev_action[self.trpy_key], atol=1e-03, rtol=0)
        ):
            self.robot.set_cartesian_position(
                common.Pose(translation=action_dict[self.trpy_key][:3], rpy_vector=action_dict[self.trpy_key][3:])
            )
        elif self.get_base_control_mode() == ControlMode.CARTESIAN_TQuat and (
            self.prev_action is None
            or not np.allclose(action_dict[self.tquat_key], self.prev_action[self.tquat_key], atol=1e-03, rtol=0)
        ):
            self.robot.set_cartesian_position(
                common.Pose(translation=action_dict[self.tquat_key][:3], quaternion=action_dict[self.tquat_key][3:])
            )
        self.prev_action = copy.deepcopy(action_dict)
        return self.get_obs(), 0, False, False, {}

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[ArmObsType, dict[str, Any]]:
        if seed is not None:
            msg = "seeding not implemented yet"
            raise NotImplementedError(msg)
        if options is not None:
            msg = "options not implemented yet"
            raise NotImplementedError(msg)
        self.robot.reset()
        if self.home_on_reset:
            self.robot.move_home()
        return self.get_obs(), {}

    def close(self):
        super().close()


class MultiRobotWrapper(gym.Env):
    """Wraps a dictionary of environments to allow for multi robot control."""

    def __init__(self, envs: dict[str, gym.Env] | dict[str, gym.Wrapper]):
        self.envs = envs
        self.unwrapped_multi = cast(dict[str, RobotEnv], {key: env.unwrapped for key, env in envs.items()})

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        # follows gym env by combinding a dict of envs into a single env
        obs = {}
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        for key, env in self.envs.items():
            obs[key], r, t, tr, info[key] = env.step(action[key])
            reward += float(r)
            terminated = terminated or t
            truncated = truncated or tr
            info[key]["terminated"] = t
            info[key]["truncated"] = tr
        return obs, reward, terminated, truncated, info

    def reset(
        self, seed: dict[str, int] | None = None, options: dict[str, dict[str, Any]] | None = None  # type: ignore
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs = {}
        info = {}

        seed_ = seed if seed is not None else {key: None for key in self.envs}  # type: ignore
        options_ = options if options is not None else {key: None for key in self.envs}  # type: ignore
        for key, env in self.envs.items():
            obs[key], info[key] = env.reset(seed=seed_[key], options=options_[key])
        return obs, info

    def get_wrapper_attr(self, name: str) -> Any:
        """Gets an attribute from the wrapper and lower environments if `name` doesn't exist in this object.
        If lower environments have the same attribute, it returns a dictionary of the attribute values.
        """
        if name in self.__dir__():
            return getattr(self, name)
        return {key: env.get_wrapper_attr(name) for key, env in self.envs.items()}

    def close(self):
        for env in self.envs.values():
            env.close()


class RelativeTo(Enum):
    LAST_STEP = auto()
    CONFIGURED_ORIGIN = auto()


class RelativeActionSpace(gym.ActionWrapper):
    DEFAULT_MAX_CART_MOV = 0.5
    DEFAULT_MAX_CART_ROT = np.deg2rad(90)
    DEFAULT_MAX_JOINT_MOV = np.deg2rad(5)

    def __init__(
        self,
        env,
        relative_to: RelativeTo = RelativeTo.LAST_STEP,
        max_mov: float | tuple[float, float] | None = None,
    ):
        super().__init__(env)
        self.unwrapped: RobotEnv
        self.action_space: gym.spaces.Dict
        self.relative_to = relative_to
        if (
            self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TRPY
            or self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TQuat
        ):
            if max_mov is None:
                max_mov = (self.DEFAULT_MAX_CART_MOV, self.DEFAULT_MAX_CART_ROT)
            elif isinstance(max_mov, float):
                _logger.info("No rotation maximum given, using default of %s rad", self.DEFAULT_MAX_CART_ROT)
                max_mov = (max_mov, self.DEFAULT_MAX_CART_ROT)
            assert (
                isinstance(max_mov, tuple) and len(max_mov) == 2
            ), "in cartesian control max_mov must be a tuple of maximum translation (in m) and maximum rotation in (rad)"
            if max_mov[0] > 1:
                _logger.warning(
                    "maximal translation movement is set to a value higher than 1m, which is really high, consider setting it lower"
                )
            if max_mov[1] > np.deg2rad(180):
                _logger.warning(
                    "maximal rotation movement is set to a value higher than 180 degree, which is really high, consider setting it lower"
                )
        else:
            # control mode is in joint space
            if max_mov is None:
                max_mov = self.DEFAULT_MAX_JOINT_MOV
            assert isinstance(
                max_mov, float
            ), "in cartesian control max_mov must be a float representing the maximum allowed rotation (in rad)."
            if max_mov > np.deg2rad(180):
                _logger.warning(
                    "maximal movement is set higher to a value higher than 180 degree, which is really high, consider setting it lower"
                )
        self.max_mov: float | tuple[float, float] = max_mov

        if self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TRPY:
            assert isinstance(self.max_mov, tuple)
            self.action_space.spaces.update(
                get_space(
                    LimitedTRPYRelDictType,
                    params={"cart_limits": {"max_cart_mov": self.max_mov[0], "max_angle_mov": self.max_mov[1]}},
                ).spaces
            )
        elif self.unwrapped.get_control_mode() == ControlMode.JOINTS:
            self.action_space.spaces.update(
                get_space(
                    LimitedJointsRelDictType,
                    params={"joint_limits": {"max_joint_mov": self.max_mov, "dof": get_dof(self.unwrapped.robot)}},
                ).spaces
            )
        elif self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TQuat:
            assert isinstance(self.max_mov, tuple)
            self.action_space.spaces.update(
                get_space(
                    LimitedTQuatRelDictType,
                    params={"cart_limits": {"max_cart_mov": self.max_mov[0]}},
                ).spaces
            )
        else:
            msg = "Control mode not recognized!"
            raise ValueError(msg)
        self.joints_key = get_space_keys(LimitedJointsRelDictType)[0]
        self.trpy_key = get_space_keys(LimitedTRPYRelDictType)[0]
        self.tquat_key = get_space_keys(LimitedTQuatRelDictType)[0]
        self.initial_obs: dict[str, Any] | None = None
        self._origin: common.Pose | VecType | None = None
        self._last_action: common.Pose | VecType | None = None

    def set_origin(self, origin: common.Pose | VecType):
        if self.unwrapped.get_control_mode() == ControlMode.JOINTS:
            assert isinstance(
                origin, np.ndarray
            ), "Invalid origin type. If control mode is joints, origin must be VecType."
            self._origin = copy.deepcopy(origin)
        else:
            assert isinstance(
                origin, common.Pose
            ), "Invalid origin type. If control mode is cartesian, origin must be Pose."
            self._origin = copy.deepcopy(origin)

    def set_origin_to_current(self):
        if self.unwrapped.get_control_mode() == ControlMode.JOINTS:
            self._origin = self.unwrapped.robot.get_joint_position()
        else:
            self._origin = self.unwrapped.robot.get_cartesian_position()

    def reset(self, **kwargs) -> tuple[dict, dict[str, Any]]:
        obs, info = super().reset(**kwargs)
        self.initial_obs = obs
        self.set_origin_to_current()
        self._last_action = None
        return obs, info

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        if self.relative_to == RelativeTo.LAST_STEP:
            # TODO: should we use the last observation instead?
            # -> could be done after the step to the state that is returned by the observation
            self.set_origin_to_current()
        action = copy.deepcopy(action)
        if self.unwrapped.get_control_mode() == ControlMode.JOINTS and self.joints_key in action:
            assert isinstance(self._origin, np.ndarray), "Invalid origin type give the control mode."
            assert isinstance(self.max_mov, float)
            low, high = get_joint_limits(self.unwrapped.robot)
            # TODO: should we also clip euqally for all joints?
            if self.relative_to == RelativeTo.LAST_STEP or self._last_action is None:
                limited_joints = np.clip(action[self.joints_key], -self.max_mov, self.max_mov)
                self._last_action = limited_joints
            else:
                joints_diff = action[self.joints_key] - self._last_action
                limited_joints_diff = np.clip(joints_diff, -self.max_mov, self.max_mov)
                limited_joints = limited_joints_diff + self._last_action
                self._last_action = limited_joints
            action.update(JointsDictType(joints=np.clip(self._origin + limited_joints, low, high)))

        elif self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TRPY and self.trpy_key in action:
            assert isinstance(self._origin, common.Pose), "Invalid origin type given the control mode."
            assert isinstance(self.max_mov, tuple)
            pose_space = cast(gym.spaces.Box, get_space(TRPYDictType).spaces[self.trpy_key])

            if self.relative_to == RelativeTo.LAST_STEP or self._last_action is None:
                clipped_pose_offset = (
                    common.Pose(
                        translation=action[self.trpy_key][:3],
                        rpy_vector=action[self.trpy_key][3:],
                    )
                    .limit_translation_length(self.max_mov[0])
                    .limit_rotation_angle(self.max_mov[1])
                )
                self._last_action = clipped_pose_offset
            else:
                assert isinstance(self._last_action, common.Pose)
                pose_diff = (
                    common.Pose(
                        translation=action[self.trpy_key][:3],
                        rpy_vector=action[self.trpy_key][3:],
                    )
                    * self._last_action.inverse()
                )
                clipped_pose_diff = pose_diff.limit_translation_length(self.max_mov[0]).limit_rotation_angle(
                    self.max_mov[1]
                )
                clipped_pose_offset = clipped_pose_diff * self._last_action
                self._last_action = clipped_pose_offset

            unclipped_pose = common.Pose(
                translation=self._origin.translation() + clipped_pose_offset.translation(),
                rpy_vector=(clipped_pose_offset * self._origin).rotation_rpy().as_vector(),
            )
            action.update(
                TRPYDictType(
                    xyzrpy=np.concatenate(
                        [
                            np.clip(unclipped_pose.translation(), pose_space.low[:3], pose_space.high[:3]),
                            unclipped_pose.rotation_rpy().as_vector(),
                        ],
                    )
                )
            )
        elif self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TQuat and self.tquat_key in action:
            assert isinstance(self._origin, common.Pose), "Invalid origin type given the control mode."
            assert isinstance(self.max_mov, tuple)
            pose_space = cast(gym.spaces.Box, get_space(TQuatDictType).spaces[self.tquat_key])

            if self.relative_to == RelativeTo.LAST_STEP or self._last_action is None:
                clipped_pose_offset = (
                    common.Pose(
                        translation=action[self.tquat_key][:3],
                        quaternion=action[self.tquat_key][3:],
                    )
                    .limit_translation_length(self.max_mov[0])
                    .limit_rotation_angle(self.max_mov[1])
                )
                self._last_action = clipped_pose_offset
            else:
                assert isinstance(self._last_action, common.Pose)
                pose_diff = (
                    common.Pose(
                        translation=action[self.tquat_key][:3],
                        quaternion=action[self.tquat_key][3:],
                    )
                    * self._last_action.inverse()
                )
                clipped_pose_diff = pose_diff.limit_translation_length(self.max_mov[0]).limit_rotation_angle(
                    self.max_mov[1]
                )
                clipped_pose_offset = clipped_pose_diff * self._last_action
                self._last_action = clipped_pose_offset

            unclipped_pose = common.Pose(
                translation=self._origin.translation() + clipped_pose_offset.translation(),
                quaternion=(clipped_pose_offset * self._origin).rotation_q(),
            )

            action.update(
                TQuatDictType(
                    tquat=np.concatenate(
                        [
                            np.clip(unclipped_pose.translation(), pose_space.low[:3], pose_space.high[:3]),
                            unclipped_pose.rotation_q(),
                        ],
                    )
                )
            )
        else:
            msg = "Given type is not matching control mode!"
            raise RuntimeError(msg)
        return action


class CameraSetWrapper(ActObsInfoWrapper):
    RGB_KEY = "rgb"
    DEPTH_KEY = "depth"

    def __init__(self, env, camera_set: BaseCameraSet, include_depth: bool = False):
        super().__init__(env)
        self.unwrapped: RobotEnv
        self.camera_set = camera_set
        self.include_depth = include_depth

        self.observation_space: gym.spaces.Dict
        # rgb is always included
        params: dict = {
            f"/{name}/{self.RGB_KEY}/frame": {
                "height": camera_set.config(name).resolution_height,
                "width": camera_set.config(name).resolution_width,
            }
            for name in camera_set.camera_names
        }
        if self.include_depth:
            # depth is optional
            params.update(
                {
                    f"/{name}/{self.DEPTH_KEY}/frame": {
                        # values metric but scaled with factor rcs.BaseCameraSet.DEPTH_SCALE to fit into uint16
                        "height": camera_set.config(name).resolution_height,
                        "width": camera_set.config(name).resolution_width,
                        "color_dim": 1,
                        "dtype": np.uint16,
                        "low": 0,
                        "high": 65535,
                    }
                    for name in camera_set.camera_names
                }
            )
        self.observation_space.spaces.update(
            get_space(
                CameraDictType,
                child_dict_keys_to_unfold={
                    "camera_names": camera_set.camera_names,
                    "camera_type": [self.RGB_KEY, self.DEPTH_KEY] if self.include_depth else [self.RGB_KEY],
                },
                params=params,
            ).spaces
        )
        self.camera_key = get_space_keys(CameraDictType)[0]

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[dict, dict[str, Any]]:
        self.camera_set.clear_buffer()
        return super().reset(seed=seed, options=options)

    def observation(self, observation: dict, info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        info = copy.deepcopy(info)
        frameset = self.camera_set.get_latest_frames()
        if frameset is None:
            observation[self.camera_key] = {}
            info["camera_available"] = False
            return observation, info

        def check_depth(depth):
            if self.include_depth and depth is None:
                msg = "Depth is not available in data but still requested."
                raise ValueError(msg)
            return self.include_depth

        frame_dict: dict[str, dict[str, CameraDataDictType]] = {
            camera_name: (
                {
                    self.RGB_KEY: CameraDataDictType(
                        data=frame.camera.color.data,
                        intrinsics=frame.camera.color.intrinsics,
                        extrinsics=frame.camera.color.extrinsics,
                    ),
                    self.DEPTH_KEY: CameraDataDictType(data=frame.camera.depth.data, intrinsics=frame.camera.depth.intrinsics, extrinsics=frame.camera.depth.extrinsics),  # type: ignore
                }
                if check_depth(frame.camera.depth)
                else {
                    self.RGB_KEY: CameraDataDictType(
                        data=frame.camera.color.data,
                        intrinsics=frame.camera.color.intrinsics,
                        extrinsics=frame.camera.color.extrinsics,
                    ),
                }
            )
            for camera_name, frame in frameset.frames.items()
        }
        observation[self.camera_key] = frame_dict

        info["camera_available"] = True
        if frameset.avg_timestamp is not None:
            info["frame_timestamp"] = frameset.avg_timestamp
        return observation, info

    def close(self):
        self.camera_set.close()
        super().close()


class GripperWrapper(ActObsInfoWrapper):
    # TODO: sticky gripper, like in aloha

    BINARY_GRIPPER_CLOSED = 0
    BINARY_GRIPPER_OPEN = 1

    def __init__(self, env, gripper: common.Gripper, binary: bool = True, open_on_reset: bool = True):
        super().__init__(env)
        self.unwrapped: RobotEnv
        self.observation_space: gym.spaces.Dict
        self.observation_space.spaces.update(get_space(GripperDictType).spaces)
        self.action_space: gym.spaces.Dict
        self.action_space.spaces.update(get_space(GripperDictType).spaces)
        self.gripper_key = get_space_keys(GripperDictType)[0]
        self._gripper = gripper
        self.binary = binary
        self._last_gripper_cmd = None
        self.open_on_reset = open_on_reset

    def reset(self, **kwargs) -> tuple[dict[str, Any], dict[str, Any]]:
        if self.open_on_reset:
            # resetting opens the gripper
            self._gripper.reset()
        self._last_gripper_cmd = None
        return super().reset(**kwargs)

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        if self.binary:
            observation[self.gripper_key] = (
                self._last_gripper_cmd if self._last_gripper_cmd is not None else self.BINARY_GRIPPER_OPEN
            )
        else:
            observation[self.gripper_key] = self._gripper.get_normalized_width()

        return observation, info

    def action(self, action: dict[str, Any]) -> dict[str, Any]:

        action = copy.deepcopy(action)
        assert self.gripper_key in action, "Gripper action not found."

        gripper_action = np.round(action[self.gripper_key]) if self.binary else action[self.gripper_key]
        gripper_action = np.clip(gripper_action, 0.0, 1.0)

        if self.binary:
            self._gripper.grasp() if gripper_action == self.BINARY_GRIPPER_CLOSED else self._gripper.open()
        else:
            self._gripper.set_normalized_width(gripper_action)
        self._last_gripper_cmd = gripper_action
        del action[self.gripper_key]
        return action


class HandWrapper(ActObsInfoWrapper):
    """
    This wrapper allows for controlling the hand of the robot
    using either binary or continuous actions.
    The binary action space allows for opening and closing the hand,
    while the continuous action space allows for setting the hand
    to a specific pose.
    The wrapper also provides an observation space that includes
    the hand state.
    The hand state is represented as a binary value (0 for closed,
    1 for open) or as a continuous value (normalized joint positions).
    """

    BINARY_HAND_CLOSED = 0
    BINARY_HAND_OPEN = 1

    def __init__(self, env, hand: BaseHand, binary: bool = True):
        super().__init__(env)
        self.unwrapped: RobotEnv
        self.observation_space: gym.spaces.Dict
        self.action_space: gym.spaces.Dict
        self.binary = binary
        if self.binary:
            self.observation_space.spaces.update(get_space(HandBinDictType).spaces)
            self.action_space.spaces.update(get_space(HandBinDictType).spaces)
            self.hand_key = get_space_keys(HandBinDictType)[0]
        else:
            self.observation_space.spaces.update(get_space(HandVecDictType).spaces)
            self.action_space.spaces.update(get_space(HandVecDictType).spaces)
            self.hand_key = get_space_keys(HandVecDictType)[0]
        self._hand = hand
        self._last_hand_cmd = None

    def reset(self, **kwargs) -> tuple[dict[str, Any], dict[str, Any]]:
        self._hand.reset()
        self._last_hand_cmd = None
        return super().reset(**kwargs)

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        if self.binary:
            observation[self.hand_key] = (
                self._last_hand_cmd if self._last_hand_cmd is not None else self.BINARY_HAND_OPEN
            )
        else:
            observation[self.hand_key] = self._hand.get_normalized_joints_poses()

        info = {}
        return observation, info

    def action(self, action: dict[str, Any]) -> dict[str, Any]:

        action = copy.deepcopy(action)
        assert self.hand_key in action, "hand action not found."

        hand_action = np.round(action[self.hand_key]) if self.binary else action[self.hand_key]
        hand_action = np.clip(hand_action, 0.0, 1.0)

        if self.binary:
            if self._last_hand_cmd is None or self._last_hand_cmd != hand_action:
                if hand_action == self.BINARY_HAND_CLOSED:
                    self._hand.grasp()
                else:
                    self._hand.open()
        else:
            self._hand.set_normalized_joints_poses(hand_action)
        self._last_hand_cmd = hand_action
        del action[self.hand_key]
        return action
