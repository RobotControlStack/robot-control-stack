"""Gym API."""

import copy
from enum import Enum, auto
from typing import Annotated, Any, TypeAlias, cast

import gymnasium as gym
import numpy as np
from rcsss import common
from rcsss.camera.interface import BaseCameraSet
from rcsss.envs.space_utils import (
    ActObsInfoWrapper,
    RCSpaceType,
    Vec6Type,
    Vec7Type,
    get_space,
    get_space_keys,
)


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
        Annotated[
            np.ndarray,
            # needs to be filled with values downstream
            lambda height, width: gym.spaces.Box(
                low=0,
                high=255,
                shape=(height, width, 3),
                dtype=np.uint8,
            ),
            "frame",
        ],
    ]


# joining works with inhertiance but need to inherit from protocol again
class ArmObsType(TQuartDictType, JointsDictType): ...


CartOrJointContType: TypeAlias = TQuartDictType | JointsDictType | TRPYDictType
LimitedCartOrJointContType: TypeAlias = LimitedTQuartRelDictType | LimitedJointsRelDictType | LimitedTRPYRelDictType


class ControlMode(Enum):
    JOINTS = 1
    CARTESIAN_TRPY = 2
    CARTESIAN_TQuart = 3


class FR3Env(gym.Env):
    """Joint Gym Environment for Franka Research 3.

    Top view of on the robot. Robot faces into x direction.
    z direction faces upwards. (Right handed coordinate axis)
        ^ x
    <-  RobotBase
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
            self.prev_action is None or not np.allclose(action_dict[self.joints_key], self.prev_action[self.joints_key])
        ):
            # cast is needed because typed dicts cannot be checked at runtime
            self.robot.set_joint_position(action_dict[self.joints_key])
        elif self.get_base_control_mode() == ControlMode.CARTESIAN_TRPY and (
            self.prev_action is None or not np.allclose(action_dict[self.trpy_key], self.prev_action[self.trpy_key])
        ):
            self.robot.set_cartesian_position(
                common.Pose(translation=action_dict[self.trpy_key][:3], rpy_vector=action_dict[self.trpy_key][3:])
            )
        elif self.get_base_control_mode() == ControlMode.CARTESIAN_TQuart and (
            self.prev_action is None or not np.allclose(action_dict[self.tquart_key], self.prev_action[self.tquart_key])
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
        return self.get_obs(), {}


class RelativeTo(Enum):
    LAST_STEP = auto()
    CONFIGURED_ORIGIN = auto()


class RelativeActionSpace(gym.ActionWrapper):
    MAX_CART_MOV = 0.5
    MAX_JOINT_MOV = np.deg2rad(5)

    def __init__(self, env, relative_to: RelativeTo = RelativeTo.LAST_STEP):
        super().__init__(env)
        self.unwrapped: FR3Env
        self.action_space: gym.spaces.Dict
        self.relative_to = relative_to
        if self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TRPY:
            self.action_space.spaces.update(
                get_space(LimitedTRPYRelDictType, params={"cart_limits": {"max_cart_mov": self.MAX_CART_MOV}}).spaces
            )
        elif self.unwrapped.get_control_mode() == ControlMode.JOINTS:
            self.action_space.spaces.update(
                get_space(
                    LimitedJointsRelDictType, params={"joint_limits": {"max_joint_mov": self.MAX_JOINT_MOV}}
                ).spaces
            )
        elif self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TQuart:
            self.action_space.spaces.update(
                get_space(LimitedTQuartRelDictType, params={"cart_limits": {"max_cart_mov": self.MAX_CART_MOV}}).spaces
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
            joint_space = cast(gym.spaces.Box, get_space(JointsDictType).spaces[self.joints_key])
            limited_joints = np.clip(action[self.joints_key], -self.MAX_JOINT_MOV, self.MAX_JOINT_MOV)
            action.update(
                JointsDictType(joints=np.clip(self._origin + limited_joints, joint_space.low, joint_space.high))
            )

        elif self.unwrapped.get_control_mode() == ControlMode.CARTESIAN_TRPY and self.trpy_key in action:
            assert isinstance(self._origin, common.Pose), "Invalid origin type given the control mode."
            pose_space = cast(gym.spaces.Box, get_space(TRPYDictType).spaces[self.trpy_key])
            clipped_translation = np.clip(action[self.trpy_key][:3], -self.MAX_CART_MOV, self.MAX_CART_MOV)
            unclipped_pose = (
                common.Pose(translation=clipped_translation, rpy_vector=action[self.trpy_key][3:]) * self._origin
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
            pose_space = cast(gym.spaces.Box, get_space(TQuartDictType).spaces[self.tquart_key])
            clipped_translation = np.clip(action[self.tquart_key][:3], -self.MAX_CART_MOV, self.MAX_CART_MOV)

            unclipped_pose_offset = common.Pose(translation=clipped_translation, quaternion=action[self.tquart_key][3:])
            unclipped_pose = common.Pose(
                translation=self._origin.translation() + unclipped_pose_offset.translation(),
                quaternion=(unclipped_pose_offset * self._origin).rotation_q(),
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
    def __init__(self, env, camera_set: BaseCameraSet):
        super().__init__(env)
        self.unwrapped: FR3Env
        self.camera_set = camera_set

        self.observation_space: gym.spaces.Dict
        self.observation_space.spaces.update(
            get_space(
                CameraDictType,
                child_dict_keys_to_unfold={"camera_names": camera_set.camera_names},
                params={
                    "frame": {
                        "height": camera_set.config.resolution_height,
                        "width": camera_set.config.resolution_height,
                    }
                },
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
        assert frameset is not None, "No frame available."
        color_frame_dict: dict[str, np.ndarray] = {
            camera_name: frame.camera.color.data for camera_name, frame in frameset.frames.items()
        }
        observation[self.camera_key] = color_frame_dict

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
        self._last_gripper_action = 1
        self.binary = binary

    def reset(self, **kwargs) -> tuple[dict[str, Any], dict[str, Any]]:
        self._gripper.reset()
        self._last_gripper_action = 1
        return super().reset(**kwargs)

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        observation[self.gripper_key] = self._gripper.get_normalized_width()
        if self.binary:
            observation[self.gripper_key] = round(observation[self.gripper_key])
        return observation, info

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        action = copy.deepcopy(action)
        assert self.gripper_key in action, "Gripper action not found."

        gripper_action = np.round(action["gripper"])
        if gripper_action != self._last_gripper_action:
            self._last_gripper_action = gripper_action
            if self.binary:
                self._gripper.grasp() if self._gripper_state == 0 else self._gripper.open()
            else:
                self._gripper.set_normalized_width(action["gripper"])
        del action[self.gripper_key]
        return action
