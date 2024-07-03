"""Gym API."""

from enum import Enum
from typing import Annotated, Any, TypeAlias, cast

import gymnasium as gym
import numpy as np
from rcsss import common
from rcsss.camera.interface import BaseCameraSet
from rcsss.envs.space_utils import (
    ObservationInfoWrapper,
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
            high=np.array([0.855, 0.855, 0.1188, np.deg2rad(180), np.deg2rad(180), np.deg2rad(180)]),
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
            high=np.array([0.855, 0.855, 0.1188] + [1] + [np.inf] * 3),
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


class FR3Env(gym.Env[ArmObsType, CartOrJointContType]):
    """Joint Gym Environment for Franka Research 3."""

    def __init__(self, robot: common.Robot, control_mode: ControlMode):
        self.robot = robot
        self.control_mode = control_mode
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

    def _get_obs(self) -> ArmObsType:
        return ArmObsType(
            tquart=np.concat(
                [self.robot.get_cartesian_position().translation(), self.robot.get_cartesian_position().rotation_q()]
            ),
            joints=self.robot.get_joint_position(),
        )

    def step(self, action: CartOrJointContType) -> tuple[ArmObsType, float, bool, bool, dict]:
        if self.control_mode == ControlMode.JOINTS and self.joints_key in action:
            self.robot.set_joint_position(action[self.joints_key])
        elif self.control_mode == ControlMode.CARTESIAN_TRPY and self.trpy_key in action:
            self.robot.set_cartesian_position(
                common.Pose(translation=action[self.trpy_key][:3], rpy_vector=action[self.trpy_key][3:])
            )
        elif self.control_mode == ControlMode.CARTESIAN_TQuart and self.tquart_key in action:
            self.robot.set_cartesian_position(
                common.Pose(translation=action[self.tquart_key][:3], quaternion=action[self.tquart_key][3:])
            )
        else:
            msg = "Given type is not matching control mode!"
            raise RuntimeError(msg)
        return self._get_obs(), 0, False, False, {}

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[ArmObsType, dict[str, Any]]:
        if seed is not None:
            msg = "seeding not implemented yet"
            raise NotImplementedError(msg)
        if options is not None:
            msg = "options not implemented yet"
            raise NotImplementedError(msg)
        return self._get_obs(), {}


class RelativeActionSpace(gym.ActionWrapper):
    MAX_CART_MOV = 0.1
    MAX_JOINT_MOV = np.deg2rad(30)

    def __init__(self, env):
        self.env: FR3Env
        super().__init__(env)
        self.action_space: gym.spaces.Dict
        if self.env.control_mode == ControlMode.CARTESIAN_TRPY:
            self.action_space.spaces.update(
                get_space(LimitedTRPYRelDictType, params={"cart_limits": {"max_cart_mov": self.MAX_CART_MOV}}).spaces
            )
        elif self.env.control_mode == ControlMode.JOINTS:
            self.action_space.spaces.update(
                get_space(
                    LimitedJointsRelDictType, params={"joint_limits": {"max_joint_mov": self.MAX_JOINT_MOV}}
                ).spaces
            )
        elif self.env.control_mode == ControlMode.CARTESIAN_TQuart:
            self.action_space.spaces.update(
                get_space(
                    LimitedTQuartRelDictType, params={"cart_limits": {"max_joint_mov": self.MAX_JOINT_MOV}}
                ).spaces
            )
        else:
            msg = "Control mode not recognized!"
            raise ValueError(msg)
        self.joints_key = get_space_keys(LimitedJointsRelDictType)[0]
        self.trpy_key = get_space_keys(LimitedTRPYRelDictType)[0]
        self.tquart_key = get_space_keys(LimitedTQuartRelDictType)[0]

    def action(self, action: LimitedCartOrJointContType) -> CartOrJointContType:
        if self.env.control_mode == ControlMode.JOINTS and self.joints_key in action:
            joint_space: gym.spaces.Box = get_space(JointsDictType).spaces[self.joints_key]
            limited_joint_space: gym.spaces.Box = get_space(LimitedJointsRelDictType).spaces[self.joints_key]
            limited_joints = np.clip(action[self.joints_key], limited_joint_space.low, limited_joint_space.high)
            return JointsDictType(
                joints=np.clip(self.env.robot.get_joint_position() + limited_joints, joint_space.low, joint_space.high)
            )

        elif self.env.control_mode == ControlMode.CARTESIAN_TRPY and self.trpy_key in action:
            pose_space: gym.spaces.Box = get_space(TRPYDictType).spaces[self.trpy_key]
            limited_pose_space: gym.spaces.Box = get_space(LimitedTRPYRelDictType).spaces[self.trpy_key]
            clipped_translation = np.clip(
                action[self.trpy_key][:3], limited_pose_space.low[:3], limited_pose_space.high[3:]
            )
            unclipped_pose = self.env.robot.get_cartesian_position() * common.Pose(
                translation=clipped_translation, rpy_vector=action[self.trpy_key][3:]
            )
            return TRPYDictType(
                xyzrpy=np.concat(
                    [np.clip(unclipped_pose.translation(), pose_space.low[:3], pose_space.high[3:]),
                    unclipped_pose.rotation_rpy().as_vector()],
                )
            )
        elif self.env.control_mode == ControlMode.CARTESIAN_TQuart and self.tquart_key in action:
            pose_space: gym.spaces.Box = get_space(TQuartDictType).spaces[self.tquart_key]
            limited_pose_space: gym.spaces.Box = get_space(LimitedTRPYRelDictType).spaces[self.trpy_key]
            clipped_translation = np.clip(
                action[self.tquart_key][:3], limited_pose_space.low[:3], limited_pose_space.high[3:]
            )
            unclipped_pose = self.env.robot.get_cartesian_position() * common.Pose(
                translation=limited_pose_space, quaternion=action[self.tquart_key][3:]
            )
            return TQuartDictType(
                tquart=np.concat(
                    [np.clip(unclipped_pose.translation(), pose_space.low[:3], pose_space.high[3:]),
                    unclipped_pose.rotation_q()],
                )
            )
        else:
            msg = "Given type is not matching control mode!"
            raise RuntimeError(msg)


class CameraSetWrapper(ObservationInfoWrapper):
    def __init__(self, env: FR3Env, camera_set: BaseCameraSet):
        self.env: FR3Env
        self.observation_space: gym.spaces.Dict
        super().__init__(env)
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

    def observation(self, observation: dict, info: dict[str, Any]) -> dict[str, Any]:
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


# TODO: sticky gripper, like in aloha
class GripperWrapper(gym.ObservationWrapper, gym.ActionWrapper):
    def __init__(self, env, gripper: common.Gripper):
        self.env: FR3Env
        super().__init__(env)
        self.observation_space: gym.spaces.Dict
        self.observation_space.spaces.update(get_space(GripperDictType).spaces)
        self.action_space: gym.spaces.Dict
        self.action_space.spaces.update(get_space(GripperDictType).spaces)
        self.gripper_key = get_space_keys(GripperDictType)[0]
        self._gripper = gripper
        self._gripper_state = True

    def reset(self, **kwargs):
        self._gripper.release()
        self._gripper_state = 1
        return super().reset(**kwargs)

    def observation(self, observation: dict[str, Any]) -> dict[str, Any]:
        observation[self.gripper_key] = self._gripper_state

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        assert self.gripper_key in action, "Gripper action not found."
        self._gripper_state = round(action["gripper"])
        self._gripper.shut() if self._gripper_state == 0 else self._gripper.release()
        del action[self.gripper_key]
        return action
