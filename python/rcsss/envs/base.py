"""Gym API."""

import copy
import logging
from enum import Enum, auto
from typing import Annotated, Any, TypeAlias, cast

import gymnasium as gym
import numpy as np
from rcsss import common, sim
from rcsss.camera.interface import BaseCameraSet
from rcsss.envs.space_utils import (
    ActObsInfoWrapper,
    RCSpaceType,
    Vec6Type,
    Vec7Type,
    get_space,
    get_space_keys,
)

_logger = logging.getLogger()


class TRPYDictType(RCSpaceType):
    """Pose format is in transpose[3],r,p,y"""

    xyzrpy: Annotated[
        Vec6Type,
        gym.spaces.Box(
            low=np.array([-0.855, -0.855, 0, -np.deg2rad(180), -np.deg2rad(180), -np.deg2rad(180)]),
            high=np.array([0.855, 0.855, 1.188, np.deg2rad(180), np.deg2rad(180), np.deg2rad(180)]),
            dtype=np.float32,
        ),
    ]


class LimitedTRPYRelDictType(RCSpaceType):
    xyzrpy: Annotated[
        Vec6Type,
        lambda max_cart_mov: gym.spaces.Box(
            low=np.array(3 * [-max_cart_mov] + 3 * [-np.deg2rad(180)]),
            high=np.array(3 * [max_cart_mov] + 3 * [np.deg2rad(180)]),
            dtype=np.float32,
        ),
        "cart_limits",
    ]


class TQuartDictType(RCSpaceType):
    tquart: Annotated[
        Vec7Type,
        gym.spaces.Box(
            low=np.array([-0.855, -0.855, 0] + [-1] + [-np.inf] * 3),
            high=np.array([0.855, 0.855, 1.188] + [1] + [np.inf] * 3),
            dtype=np.float32,
        ),
    ]


class LimitedTQuartRelDictType(RCSpaceType):
    tquart: Annotated[
        Vec7Type,
        lambda max_cart_mov: gym.spaces.Box(
            low=np.array(3 * [-max_cart_mov] + [-1] + [-np.inf] * 3),
            high=np.array(3 * [max_cart_mov] + [1] + [np.inf] * 3),
            dtype=np.float32,
        ),
        "cart_limits",
    ]


class JointsDictType(RCSpaceType):
    joints: Annotated[
        Vec7Type,
        gym.spaces.Box(
            low=np.array([-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895]),
            high=np.array([2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895]),
            dtype=np.float32,
        ),
    ]


class LimitedJointsRelDictType(RCSpaceType):
    joints: Annotated[
        Vec7Type,
        lambda max_joint_mov: gym.spaces.Box(
            low=np.array(7 * [-max_joint_mov]),
            high=np.array(7 * [max_joint_mov]),
            dtype=np.float32,
        ),
        "joint_limits",
    ]


class GripperDictType(RCSpaceType):
    # 0 for closed, 1 for open (>=0.5 for open)
    gripper: Annotated[float, gym.spaces.Box(low=0, high=1, dtype=np.float32)]


class CameraDictType(RCSpaceType):
    frames: dict[
        Annotated[str, "camera_names"],
        dict[
            Annotated[str, "camera_type"],  # "rgb" or "depth"
            Annotated[
                np.ndarray,
                # needs to be filled with values downstream
                lambda height, width, color_dim=3, dtype=np.uint8, low=0, high=255: gym.spaces.Box(
                    low=low,
                    high=high,
                    shape=(height, width, color_dim),
                    dtype=dtype,
                ),
                "frame",
            ],
        ],
    ]


# joining works with inheritance but need to inherit from protocol again
class ArmObsType(TQuartDictType, JointsDictType, TRPYDictType): ...


CartOrJointContType: TypeAlias = TQuartDictType | JointsDictType | TRPYDictType
LimitedCartOrJointContType: TypeAlias = LimitedTQuartRelDictType | LimitedJointsRelDictType | LimitedTRPYRelDictType


class ObsArmsGr(ArmObsType, GripperDictType):
    pass


class ObsArmsGrCam(ArmObsType, GripperDictType, CameraDictType):
    pass


class ObsArmsGrCamCG(ArmObsType, GripperDictType, CameraDictType):
    pass


class ControlMode(Enum):
    JOINTS = auto()
    CARTESIAN_TRPY = auto()
    CARTESIAN_TQuart = auto()


class RobotInstance(Enum):
    HARDWARE = auto()
    SIMULATION = auto()


class FR3Env(gym.Env):
    """Joint Gym Environment for Franka Research 3.

    Top view of on the robot. Robot faces into x direction.
    z direction faces upwards. (Right handed coordinate axis)
        ^ x
        |
    <-- RobotBase
    y
    """

    def __init__(self, robot: common.Robot, control_mode: ControlMode):
        self.robot = robot
        self._control_mode_overrides = [control_mode]
        self.action_space: gym.spaces.Dict
        self.observation_space: gym.spaces.Dict
        if control_mode == ControlMode.JOINTS:
            self.action_space = get_space(JointsDictType)
        elif control_mode == ControlMode.CARTESIAN_TRPY:
            self.action_space = get_space(TRPYDictType)
        elif control_mode == ControlMode.CARTESIAN_TQuart:
            self.action_space = get_space(TQuartDictType)
        else:
            msg = "Control mode not recognized!"
            raise ValueError(msg)
        self.observation_space = get_space(ArmObsType)
        self.joints_key = get_space_keys(JointsDictType)[0]
        self.trpy_key = get_space_keys(TRPYDictType)[0]
        self.tquart_key = get_space_keys(TQuartDictType)[0]
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
            tquart=np.concat(
                [self.robot.get_cartesian_position().translation(), self.robot.get_cartesian_position().rotation_q()]
            ),
            joints=self.robot.get_joint_position(),
            xyzrpy=self.robot.get_cartesian_position().xyzrpy(),
        )

    def step(self, action: CartOrJointContType) -> tuple[ArmObsType, float, bool, bool, dict]:
        action_dict = cast(dict, action)
        if (
            self.get_base_control_mode() == ControlMode.CARTESIAN_TQuart
            and self.tquart_key not in action_dict
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
            # cast is needed because typed dicts cannot be checked at runtime
            self.robot.set_joint_position(action_dict[self.joints_key])
        elif self.get_base_control_mode() == ControlMode.CARTESIAN_TRPY and (
            self.prev_action is None
            or not np.allclose(action_dict[self.trpy_key], self.prev_action[self.trpy_key], atol=1e-03, rtol=0)
        ):
            self.robot.set_cartesian_position(
                common.Pose(translation=action_dict[self.trpy_key][:3], rpy_vector=action_dict[self.trpy_key][3:])
            )
        elif self.get_base_control_mode() == ControlMode.CARTESIAN_TQuart and (
            self.prev_action is None
            or not np.allclose(action_dict[self.tquart_key], self.prev_action[self.tquart_key], atol=1e-03, rtol=0)
        ):
            self.robot.set_cartesian_position(
                common.Pose(translation=action_dict[self.tquart_key][:3], quaternion=action_dict[self.tquart_key][3:])
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
        return self.get_obs(), {}


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
        relative_to: RelativeTo = RelativeTo.CONFIGURED_ORIGIN,
        max_mov: float | tuple[float, float] | None = None,
    ):
        super().__init__(env)
        self.unwrapped: FR3Env
        self.action_space: gym.spaces.Dict
        self.relative_to = relative_to
        if (
            self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TRPY
            or self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TQuart
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
            self.action_space.spaces.update(
                get_space(LimitedTRPYRelDictType, params={"cart_limits": {"max_cart_mov": self.max_mov}}).spaces
            )
        elif self.unwrapped.get_control_mode() == ControlMode.JOINTS:
            self.action_space.spaces.update(
                get_space(LimitedJointsRelDictType, params={"joint_limits": {"max_joint_mov": self.max_mov}}).spaces
            )
        elif self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TQuart:
            self.action_space.spaces.update(
                get_space(LimitedTQuartRelDictType, params={"cart_limits": {"max_cart_mov": self.max_mov}}).spaces
            )
        else:
            msg = "Control mode not recognized!"
            raise ValueError(msg)
        self.joints_key = get_space_keys(LimitedJointsRelDictType)[0]
        self.trpy_key = get_space_keys(LimitedTRPYRelDictType)[0]
        self.tquart_key = get_space_keys(LimitedTQuartRelDictType)[0]
        self.initial_obs: dict[str, Any] | None = None
        self._origin: common.Pose | Vec7Type | None = None

    def set_origin(self, origin: common.Pose | Vec7Type):
        if self.unwrapped.get_control_mode() == ControlMode.JOINTS:
            assert isinstance(
                origin, np.ndarray
            ), "Invalid origin type. If control mode is joints, origin must be Vec7Type."
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
        return obs, info

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        if self.relative_to == RelativeTo.LAST_STEP:
            # TODO: should we use the last observation instead?
            self.set_origin_to_current()
        action = copy.deepcopy(action)
        if self.unwrapped.get_control_mode() == ControlMode.JOINTS and self.joints_key in action:
            assert isinstance(self._origin, np.ndarray), "Invalid origin type give the control mode."
            assert isinstance(self.max_mov, float)
            joint_space = cast(gym.spaces.Box, get_space(JointsDictType).spaces[self.joints_key])
            limited_joints = np.clip(action[self.joints_key], -self.max_mov, self.max_mov)
            action.update(
                JointsDictType(joints=np.clip(self._origin + limited_joints, joint_space.low, joint_space.high))
            )

        elif self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TRPY and self.trpy_key in action:
            assert isinstance(self._origin, common.Pose), "Invalid origin type given the control mode."
            assert isinstance(self.max_mov, tuple)
            pose_space = cast(gym.spaces.Box, get_space(TRPYDictType).spaces[self.trpy_key])

            # clip translation
            translation = action[self.trpy_key][:3]
            if np.linalg.norm(translation) > self.max_mov[0]:
                translation = translation / np.linalg.norm(translation) * self.max_mov[0]

            # clip rotation
            rotation = common.Pose(rpy=action[self.trpy_key][3:])
            if rotation.total_angle() > self.max_mov[1]:
                rotation = rotation.set_angle(self.max_mov[1])

            clipped_pose_offset = common.Pose(translation=translation, quaternion=rotation.rotation_q())
            unclipped_pose = common.Pose(
                translation=self._origin.translation() + clipped_pose_offset.translation(),
                rpy_vector=(clipped_pose_offset * self._origin).rotation_rpy().as_vector(),
            )
            action.update(
                TRPYDictType(
                    xyzrpy=np.concat(
                        [
                            np.clip(unclipped_pose.translation(), pose_space.low[:3], pose_space.high[:3]),
                            unclipped_pose.rotation_rpy().as_vector(),
                        ],
                    )
                )
            )
        elif self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TQuart and self.tquart_key in action:
            assert isinstance(self._origin, common.Pose), "Invalid origin type given the control mode."
            assert isinstance(self.max_mov, tuple)
            pose_space = cast(gym.spaces.Box, get_space(TQuartDictType).spaces[self.tquart_key])

            # clip translation
            translation = action[self.trpy_key][:3]
            if np.linalg.norm(translation) > self.max_mov[0]:
                translation = translation / np.linalg.norm(translation) * self.max_mov[0]

            # clip rotation
            rotation = common.Pose(rpy=action[self.trpy_key][3:])
            if rotation.total_angle() > self.max_mov[1]:
                rotation = rotation.set_angle(self.max_mov[1])

            clipped_pose_offset = common.Pose(translation=translation, quaternion=rotation.rotation_q())
            unclipped_pose = common.Pose(
                translation=self._origin.translation() + clipped_pose_offset.translation(),
                quaternion=(clipped_pose_offset * self._origin).rotation_q(),
            )

            action.update(
                TQuartDictType(
                    tquart=np.concat(
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
        self.unwrapped: FR3Env
        self.camera_set = camera_set
        self.include_depth = include_depth

        self.observation_space: gym.spaces.Dict
        # rgb is always included
        params: dict = {
            "frame": {
                "height": camera_set.config.resolution_height,
                "width": camera_set.config.resolution_width,
            }
        }
        if self.include_depth:
            # depth is optional
            params.update(
                {
                    f"/{name}/{self.DEPTH_KEY}/frame": {
                        "height": camera_set.config.resolution_height,
                        "width": camera_set.config.resolution_width,
                        "color_dim": 1,
                        "dtype": np.float32,
                        "low": 0.0,
                        "high": 1.0,
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

        frame_dict: dict[str, dict[str, np.ndarray]] = {
            camera_name: (
                {
                    self.RGB_KEY: frame.camera.color.data,
                    self.DEPTH_KEY: frame.camera.depth.data,  # type: ignore
                }
                if check_depth(frame.camera.depth)
                else {
                    self.RGB_KEY: frame.camera.color.data,
                }
            )
            for camera_name, frame in frameset.frames.items()
        }
        observation[self.camera_key] = frame_dict

        info["camera_available"] = True
        if frameset.avg_timestamp is not None:
            info["frame_timestamp"] = frameset.avg_timestamp
        return observation, info


class GripperWrapper(ActObsInfoWrapper):
    # TODO: sticky gripper, like in aloha
    def __init__(self, env, gripper: common.Gripper, binary: bool = True):
        super().__init__(env)
        self.unwrapped: FR3Env
        self.observation_space: gym.spaces.Dict
        self.observation_space.spaces.update(get_space(GripperDictType).spaces)
        self.action_space: gym.spaces.Dict
        self.action_space.spaces.update(get_space(GripperDictType).spaces)
        self.gripper_key = get_space_keys(GripperDictType)[0]
        self._gripper = gripper
        self.binary = binary

    def reset(self, **kwargs) -> tuple[dict[str, Any], dict[str, Any]]:
        self._gripper.reset()
        return super().reset(**kwargs)

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        observation[self.gripper_key] = self._gripper.get_normalized_width()

        # TODO: a cleaner solution would be to put this code into env/sim.py
        # similar to sim fr3 has also a sim specific wrapper
        if isinstance(self._gripper, sim.FrankaHand):
            state = self._gripper.get_state()
            info["collision"] = state.collision
        if self.binary:
            observation[self.gripper_key] = round(observation[self.gripper_key])
        return observation, info

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        action = copy.deepcopy(action)
        assert self.gripper_key in action, "Gripper action not found."

        gripper_action = np.round(action["gripper"])
        width = self._gripper.get_normalized_width()
        if self.binary:
            width = np.round(width)
        if gripper_action != width:
            if self.binary:
                self._gripper.grasp() if gripper_action == 0 else self._gripper.open()
            else:
                self._gripper.set_normalized_width(action["gripper"])
        del action[self.gripper_key]
        return action
