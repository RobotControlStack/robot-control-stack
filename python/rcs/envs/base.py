"""Gym API."""

import copy
import logging
from enum import Enum, auto
from typing import Annotated, Any, ClassVar, Literal, TypeAlias, cast

import gymnasium as gym
import numpy as np
from greenlet import getcurrent, greenlet
from rcs._core.common import Hand, RobotPlatform
from rcs.camera.interface import BaseCameraSet
from rcs.envs.space_utils import (
    ActObsInfoWrapper,
    RCSpaceType,
    Vec1Type,
    Vec6Type,
    Vec7Type,
    Vec18Type,
    VecType,
    get_space,
    get_space_keys,
)
from rcs.utils import SimpleFrameRate

from rcs import common
from rcs import sim as simulation

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
    gripper: Annotated[Vec1Type, gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)]


class HandBinDictType(RCSpaceType):
    # 0 for closed, 1 for open (>=0.5 for open)
    gripper: Annotated[Vec1Type, gym.spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)]


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


class ArmWithGripper(TQuatDictType, GripperDictType): ...


class ControlMode(Enum):
    JOINTS = auto()
    CARTESIAN_TRPY = auto()
    CARTESIAN_TQuat = auto()


class BaseEnv(gym.Env):
    PLATFORM: RobotPlatform

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], float, bool, bool, dict]:
        return {}, 0, False, False, {}

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        super().reset(seed=seed, options=options)
        return {}, {}


class HardwareEnv(BaseEnv):
    PLATFORM = RobotPlatform.HARDWARE


class SimEnv(BaseEnv):
    PLATFORM = RobotPlatform.SIMULATION

    def __init__(self, sim: simulation.Sim) -> None:
        self.sim = sim
        cfg = self.sim.get_config()
        self.frame_rate = SimpleFrameRate(cfg.frequency, "MoJoCo Simulation Loop")
        self.main_greenlet: greenlet | None = None

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], float, bool, bool, dict]:
        if self.main_greenlet is not None:
            self.main_greenlet.switch()
        else:
            self.step_sim()
        return super().step(action)

    def step_sim(self):
        cfg = self.sim.get_config()
        if cfg.async_control:
            self.sim.step(round(1 / cfg.frequency / self.sim.model.opt.timestep))
            if cfg.realtime:
                self.frame_rate.frame_rate = cfg.frequency
                self.frame_rate()
        else:
            self.sim.step_until_convergence()

    def apply_sim_state(self):
        self.sim.step(2)

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        if self.main_greenlet is not None:
            self.main_greenlet.switch()
        else:
            self.apply_sim_state()
        return super().reset(seed=seed, options=options)


class CoverWrapper(gym.Wrapper):
    """The CoverWrapper must be the last wrapper on the stack

    Only strictly necessary for simulator environments, but also works for hardware environments.
    It takes care of resetting the simulator before any other wrapper resets its state, already assuming
    a fresh simulator state.
    """

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        if self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.SIMULATION:
            sim = cast(simulation.Sim, self.get_wrapper_attr("sim"))
            sim.reset()
        return super().reset(seed=seed, options=options)


class RobotWrapper(ActObsInfoWrapper):
    """Gym Wrapper for a single robot arm.

    Top view of on the robot. Robot faces into x direction.
    z direction faces upwards. (Right handed coordinate axis)
        ^ x
        |
    <-- RobotBase
    y
    """

    def __init__(self, env, robot: common.Robot, control_mode: ControlMode, home_on_reset: bool = True):
        super().__init__(env)
        self.robot = robot
        self._control_mode_overrides = [control_mode]
        self.home_on_reset = home_on_reset
        self.action_space: gym.spaces.Dict
        self.observation_space: gym.spaces.Dict
        low, high = robot.get_config().joint_limits
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

    def get_robot_obs(self) -> ArmObsType:
        return ArmObsType(
            tquat=np.concatenate(
                [self.robot.get_cartesian_position().translation(), self.robot.get_cartesian_position().rotation_q()]  # type: ignore
            ),
            joints=self.robot.get_joint_position(),
            xyzrpy=self.robot.get_cartesian_position().xyzrpy(),
        )

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        if (
            self.get_base_control_mode() == ControlMode.CARTESIAN_TQuat
            and self.tquat_key not in action
            or self.get_base_control_mode() == ControlMode.CARTESIAN_TRPY
            and self.trpy_key not in action
            or self.get_base_control_mode() == ControlMode.JOINTS
            and self.joints_key not in action
        ):
            msg = "Given type is not matching control mode!"
            raise RuntimeError(msg)
        last_action = self.prev_action
        self.prev_action = copy.deepcopy(action)

        # shallow copy
        action = dict(action)
        if self.get_base_control_mode() == ControlMode.JOINTS and (
            last_action is None
            or not np.allclose(action[self.joints_key], last_action[self.joints_key], atol=1e-03, rtol=0)
        ):
            self.robot.set_joint_position(action[self.joints_key])
            action.pop(self.joints_key)
        elif self.get_base_control_mode() == ControlMode.CARTESIAN_TRPY and (
            last_action is None
            or not np.allclose(action[self.trpy_key], last_action[self.trpy_key], atol=1e-03, rtol=0)
        ):
            self.robot.set_cartesian_position(
                common.Pose(translation=action[self.trpy_key][:3], rpy_vector=action[self.trpy_key][3:])
            )
            action.pop(self.trpy_key)
        elif self.get_base_control_mode() == ControlMode.CARTESIAN_TQuat and (
            last_action is None
            or not np.allclose(action[self.tquat_key], last_action[self.tquat_key], atol=1e-03, rtol=0)
        ):
            self.robot.set_cartesian_position(
                common.Pose(translation=action[self.tquat_key][:3], quaternion=action[self.tquat_key][3:])
            )
            action.pop(self.tquat_key)
        return action

    def observation(self, observation: dict, info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation.update(self.get_robot_obs())
        return observation, info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self.prev_action = None
        self.robot.reset()
        if self.home_on_reset:
            self.robot.move_home()
        return super().reset(seed=seed, options=options)

    def close(self):
        self.robot.close()
        super().close()


class MultiRobotWrapper(gym.Env):
    """Wraps a dictionary of single robot environments to allow for multi robot control.

    Env1  Env2  Env3
      |     |     |
    ----------------
    MultiRobotWrapper

    All envs are stepped sequentially. Supports offset of robot bases by the `robot_to_shared_base_frame` parameter.
    """

    PLATFORM: RobotPlatform | None = None

    def __init__(
        self,
        envs: dict[str, gym.Env] | dict[str, gym.Wrapper],
        robot_to_shared_base_frame: dict[str, common.Pose] | None = None,
    ):
        self.envs = envs
        self.action_space = gym.spaces.Dict({key: copy.deepcopy(env.action_space) for key, env in self.envs.items()})
        self.observation_space = gym.spaces.Dict(
            {key: copy.deepcopy(env.observation_space) for key, env in self.envs.items()}
        )
        if robot_to_shared_base_frame is None:
            self.robot_to_shared_base_frame = {}
        else:
            self.robot_to_shared_base_frame = robot_to_shared_base_frame
        self.lead_env: gym.Env | None = None
        self.sim: simulation.Sim | None = None

        # make sure all envs are the same type (sim/real)
        for env in self.envs:
            if self.PLATFORM is None:
                self.PLATFORM = self.envs[env].get_wrapper_attr("PLATFORM")
                self.lead_env = self.envs[env].unwrapped
            else:
                assert (
                    self.envs[env].get_wrapper_attr("PLATFORM") == self.PLATFORM
                ), "all envs must have the same platform!"
        self._runs_in_sim = self.PLATFORM == RobotPlatform.SIMULATION
        if self._runs_in_sim:
            self._inject_main_greenlet()
            assert isinstance(self.lead_env, SimEnv), "something is wrong with the env, the base should be type SimEnv"
            self.sim = self.lead_env.get_wrapper_attr("sim")

    def _inject_main_greenlet(self):
        main_gr = getcurrent()
        for env_item in self.envs.values():
            assert isinstance(
                env_item.unwrapped, SimEnv
            ), "something is wrong with the env, the base should be type SimEnv"
            env_item.unwrapped.main_greenlet = main_gr

    def _translate_pose(self, key, dic, to_world=True):
        r2w = self.robot_to_shared_base_frame.get(key, common.Pose())
        if not to_world:
            r2w = r2w.inverse()
        if "tquat" in dic:
            p = r2w * common.Pose(translation=dic["tquat"][:3], quaternion=dic["tquat"][3:]) * r2w.inverse()
            dic["tquat"] = np.concatenate([p.translation(), p.rotation_q()])
        if "xyzrpy" in dic:
            p = r2w * common.Pose(translation=dic["xyzrpy"][:3], rpy_vector=dic["xyzrpy"][3:]) * r2w.inverse()
            dic["xyzrpy"] = p.xyzrpy()

        return dic

    def step(self, action: dict[str, Any]) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        step_greenlets = {}
        if self._runs_in_sim:
            # SIM path: 1. DOWN: Set actions for all robots
            for key, env in self.envs.items():

                def make_step_gr(env_to_step):
                    return greenlet(env_to_step.step)

                gr = make_step_gr(env)
                step_greenlets[key] = gr

                # Translate action
                act = self._translate_pose(key, action[key], to_world=False)

                # Switch to robot greenlet. It will run until RobotSimWrapper.step switches back.
                gr.switch(act)

            # SIM path: 2. SIM: Step physics once
            assert isinstance(self.lead_env, SimEnv)
            self.lead_env.step_sim()

        # follows gym env by combinding a dict of envs into a single env
        obs = {}
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        for key, env in self.envs.items():

            if self._runs_in_sim:
                # SIM path: 3. UP: Gather observations
                # Resume robot greenlet. It returns the step results.
                ob, r, t, tr, info[key] = step_greenlets[key].switch()
            else:
                # HARDWARE path
                act = self._translate_pose(key, action[key], to_world=False)
                ob, r, t, tr, info[key] = env.step(act)

            obs[key] = self._translate_pose(key, ob, to_world=True)
            reward += float(r)
            terminated = terminated or t
            truncated = truncated or tr
            info[key]["terminated"] = t
            info[key]["truncated"] = tr
        return obs, reward, terminated, truncated, info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs = {}
        info = {}

        seed_: dict[str, int | None] = (
            {key: seed for key in self.envs} if seed is not None else {key: None for key in self.envs}
        )
        options_ = options if options is not None else {key: None for key in self.envs}

        reset_greenlets = {}
        if self._runs_in_sim:
            # SIM path: 1. DOWN: Reset each robot
            for key, env in self.envs.items():

                def make_reset_gr(env_to_reset, s, o):
                    return greenlet(lambda: env_to_reset.reset(seed=s, options=o))

                gr = make_reset_gr(env, seed_[key], options_[key])
                reset_greenlets[key] = gr
                gr.switch()

            # SIM path: 2. SIM: apply state from rested wrappers
            assert isinstance(self.lead_env, SimEnv)
            self.lead_env.apply_sim_state()

        for key, env in self.envs.items():
            if self._runs_in_sim:
                # SIM path: 3. UP: Gather initial obs
                ob, i = reset_greenlets[key].switch()
            else:
                # HARDWARE path
                ob, i = env.reset(seed=seed_[key], options=options_[key])

            obs[key] = self._translate_pose(key, ob, to_world=True)
            info[key] = i

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
    NONE = auto()


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
        self.action_space: gym.spaces.Dict
        self.relative_to = relative_to
        self._robot = cast(common.Robot, self.get_wrapper_attr("robot"))
        if (
            self.get_wrapper_attr("get_control_mode")() == ControlMode.CARTESIAN_TRPY
            or self.get_wrapper_attr("get_control_mode")() == ControlMode.CARTESIAN_TQuat
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
            ), "in joint control max_mov must be a float representing the maximum allowed rotation (in rad)."
            if max_mov > np.deg2rad(180):
                _logger.warning(
                    "maximal movement is set higher to a value higher than 180 degree, which is really high, consider setting it lower"
                )
        self.max_mov: float | tuple[float, float] = max_mov

        if self.get_wrapper_attr("get_control_mode")() == ControlMode.CARTESIAN_TRPY:
            assert isinstance(self.max_mov, tuple)
            self.action_space.spaces.update(
                get_space(
                    LimitedTRPYRelDictType,
                    params={"cart_limits": {"max_cart_mov": self.max_mov[0], "max_angle_mov": self.max_mov[1]}},
                ).spaces
            )
        elif self.get_wrapper_attr("get_control_mode")() == ControlMode.JOINTS:
            self.action_space.spaces.update(
                get_space(
                    LimitedJointsRelDictType,
                    params={"joint_limits": {"max_joint_mov": self.max_mov, "dof": self._robot.get_config().dof}},
                ).spaces
            )
        elif self.get_wrapper_attr("get_control_mode")() == ControlMode.CARTESIAN_TQuat:
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
        if self.get_wrapper_attr("get_control_mode")() == ControlMode.JOINTS:
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
        if self.get_wrapper_attr("get_control_mode")() == ControlMode.JOINTS:
            self._origin = self._robot.get_joint_position()
        else:
            self._origin = self._robot.get_cartesian_position()

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
        if self.get_wrapper_attr("get_control_mode")() == ControlMode.JOINTS and self.joints_key in action:
            assert isinstance(self._origin, np.ndarray), "Invalid origin type give the control mode."
            assert isinstance(self.max_mov, float)
            low, high = self._robot.get_config().joint_limits
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

        elif self.get_wrapper_attr("get_control_mode")() == ControlMode.CARTESIAN_TRPY and self.trpy_key in action:
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
                translation=self._origin.translation() + clipped_pose_offset.translation(),  # type: ignore
                rpy_vector=(clipped_pose_offset * self._origin).rotation_rpy().as_vector(),
            )
            action.update(
                TRPYDictType(
                    xyzrpy=np.concatenate(  # type: ignore
                        [
                            np.clip(unclipped_pose.translation(), pose_space.low[:3], pose_space.high[:3]),
                            unclipped_pose.rotation_rpy().as_vector(),
                        ],
                    )
                )
            )
        elif self.get_wrapper_attr("get_control_mode")() == ControlMode.CARTESIAN_TQuat and self.tquat_key in action:
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
                translation=self._origin.translation() + clipped_pose_offset.translation(),  # type: ignore
                quaternion=(clipped_pose_offset * self._origin).rotation_q(),
            )

            action.update(
                TQuatDictType(
                    tquat=np.concatenate(  # type: ignore
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

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[dict, dict[str, Any]]:
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

    BINARY_GRIPPER_CLOSED: ClassVar[list[float]] = [0]
    BINARY_GRIPPER_OPEN: ClassVar[list[float]] = [1]

    def __init__(self, env, gripper: common.Gripper, binary: bool = True):
        super().__init__(env)
        self.binary = binary
        self.observation_space: gym.spaces.Dict
        self.observation_space.spaces.update(get_space(GripperDictType).spaces)
        self.action_space: gym.spaces.Dict
        self.action_space.spaces.update(get_space(GripperDictType).spaces)
        self.gripper_key = get_space_keys(GripperDictType)[0]
        self.gripper = gripper
        self._last_gripper_cmd = None

    def close(self):
        self.gripper.close()
        super().close()

    def reset(self, **kwargs) -> tuple[dict[str, Any], dict[str, Any]]:
        self.gripper.reset()
        self._last_gripper_cmd = None
        return super().reset(**kwargs)

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        if self.binary:
            observation[self.gripper_key] = (
                self._last_gripper_cmd if self._last_gripper_cmd is not None else self.BINARY_GRIPPER_OPEN
            )
        else:
            observation[self.gripper_key] = [self.gripper.get_normalized_width()]

        return observation, info

    def action(self, action: dict[str, Any]) -> dict[str, Any]:

        action = copy.deepcopy(action)
        assert self.gripper_key in action, "Gripper action not found."

        gripper_action = action[self.gripper_key]
        if isinstance(gripper_action, int | float):
            gripper_action = [gripper_action]  # type: ignore
        if self.binary:
            gripper_action = np.round(gripper_action)
        gripper_action = np.clip(gripper_action, 0.0, 1.0)

        if self.binary:
            self.gripper.grasp() if gripper_action == self.BINARY_GRIPPER_CLOSED else self.gripper.open()
        else:
            self.gripper.set_normalized_width(gripper_action[0])
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

    def __init__(self, env, hand: Hand, binary: bool = True):
        super().__init__(env)
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
        self.hand = hand
        self._last_hand_cmd = None

    def reset(self, **kwargs) -> tuple[dict[str, Any], dict[str, Any]]:
        self.hand.reset()
        self._last_hand_cmd = None
        return super().reset(**kwargs)

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        observation = copy.deepcopy(observation)
        if self.binary:
            observation[self.hand_key] = (
                self._last_hand_cmd if self._last_hand_cmd is not None else self.BINARY_HAND_OPEN
            )
        else:
            observation[self.hand_key] = self.hand.get_normalized_joint_poses()

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
                    self.hand.grasp()
                else:
                    self.hand.open()
        else:
            self.hand.set_normalized_joint_poses(hand_action)
        self._last_hand_cmd = hand_action
        del action[self.hand_key]
        return action

    def close(self):
        self.hand.close()
