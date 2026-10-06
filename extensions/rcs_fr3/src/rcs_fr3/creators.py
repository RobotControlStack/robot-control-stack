import logging
import typing
from dataclasses import dataclass, field

import gymnasium as gym
import numpy as np
from rcs._core.common import Gripper, GripperConfig, HandConfig, Kinematics, Pose
from rcs.camera.hw import HardwareCameraCreatorConfig, create_hardware_camera_set
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    HandWrapper,
    HardwareEnv,
    LimitedAbsoluteAction,
    MultiRobotWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
)
from rcs.envs.scenes import RCSEnvCreator, WrapperConfig
from rcs_fr3._core import hw
from rcs_fr3.envs import FR3HW

import rcs
from frankik import FrankaKinematics

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class FrankIK(Kinematics):
    def __init__(self, global_solution: bool = False):
        Kinematics.__init__(self)
        self.global_solution = global_solution
        self.kin = FrankaKinematics(robot_type="fr3")

    def forward(self, q0: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]], tcp_offset: Pose) -> Pose:  # type: ignore
        print("forward called")
        return Pose(pose_matrix=self.kin.forward(q0, tcp_offset.pose_matrix()))

    def inverse(  # type: ignore
        self, pose: Pose, q0: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]], tcp_offset: Pose
    ) -> np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]] | None:
        return self.kin.inverse(pose.pose_matrix(), q0, tcp_offset.pose_matrix(), global_solution=self.global_solution)


# FYI: this needs to be in global namespace to avoid auto garbage collection issues
# pybind11 3.x would avoid this but with smart_holder but we cannot update due to the subfiles issue yet
FastIK = FrankIK()


def create_franka_gripper(cfg: GripperConfig) -> Gripper:
    if not isinstance(cfg, hw.FHConfig):
        msg = f"Expected rcs_fr3 FHConfig for the franka hand, got {type(cfg).__module__}.{type(cfg).__qualname__}"
        raise TypeError(msg)
    return hw.FrankaHand(cfg)


@dataclass(kw_only=True)
class FR3HardwareEnvCreatorConfig:
    robot_cfg: hw.FR3Config
    control_mode: ControlMode
    gripper_cfg: GripperConfig | None = None
    hand_cfg: HandConfig | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    frequency: float | None = None
    """Control frequency in Hz, rate limits env.step(). None disables rate limiting."""
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


@dataclass(kw_only=True)
class FR3MultiHardwareEnvCreatorConfig:
    robot_cfgs: dict[str, hw.FR3Config]
    control_mode: ControlMode
    gripper_cfgs: dict[str, GripperConfig | None] | None = None
    hand_cfgs: dict[str, HandConfig | None] | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    robot_to_shared_base_frame: dict[str, rcs.common.Pose] | None = None
    frequency: float | None = None
    """Control frequency in Hz, rate limits env.step(). None disables rate limiting."""
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSFR3ConfigEnvCreator(RCSEnvCreator[FR3HardwareEnvCreatorConfig]):
    def create_env(self, cfg: FR3HardwareEnvCreatorConfig) -> gym.Env:
        ik = rcs.common.Pin(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = hw.Franka(cfg.robot_cfg, ik)

        env: gym.Env = HardwareEnv(frequency=cfg.frequency)
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)
        env = FR3HW(env)
        if cfg.hand_cfg is not None:
            hand = rcs.registry.HANDS.get(cfg.hand_cfg.hand_type.id)(cfg.hand_cfg)
            env = HandWrapper(env, hand, binary=cfg.wrapper_cfg.binary_gripper)
        elif cfg.gripper_cfg is not None:
            gripper = rcs.registry.GRIPPERS.get(cfg.gripper_cfg.gripper_type.id)(cfg.gripper_cfg)
            env = GripperWrapper(env, gripper, binary=cfg.wrapper_cfg.binary_gripper)

        camera_set = create_hardware_camera_set(cfg.camera_cfgs)
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set, cfg.wrapper_cfg.include_depth)

        if cfg.relative_to != RelativeTo.NONE:
            env = RelativeActionSpace(env, max_mov=cfg.max_relative_movement, relative_to=cfg.relative_to)
        else:
            env = LimitedAbsoluteAction(env, max_mov=cfg.max_relative_movement)
        return CoverWrapper(env)

    def config(self) -> FR3HardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)


class RCSFR3MultiConfigEnvCreator(RCSEnvCreator[FR3MultiHardwareEnvCreatorConfig]):
    def create_env(self, cfg: FR3MultiHardwareEnvCreatorConfig) -> gym.Env:
        envs: dict[str, gym.Env] = {}
        for robot_name, robot_cfg in cfg.robot_cfgs.items():
            envs[robot_name] = RCSFR3ConfigEnvCreator().create_env(
                FR3HardwareEnvCreatorConfig(
                    robot_cfg=robot_cfg,
                    control_mode=cfg.control_mode,
                    gripper_cfg=cfg.gripper_cfgs[robot_name] if cfg.gripper_cfgs is not None else None,
                    hand_cfg=cfg.hand_cfgs[robot_name] if cfg.hand_cfgs is not None else None,
                    camera_cfgs=None,
                    max_relative_movement=cfg.max_relative_movement,
                    relative_to=cfg.relative_to,
                    frequency=cfg.frequency,
                    wrapper_cfg=cfg.wrapper_cfg,
                )
            )

        env: gym.Env = MultiRobotWrapper(envs, cfg.robot_to_shared_base_frame)
        camera_set = create_hardware_camera_set(cfg.camera_cfgs)
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set, cfg.wrapper_cfg.include_depth)
        return CoverWrapper(env)

    def config(self) -> FR3MultiHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
