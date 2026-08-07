import logging
from dataclasses import dataclass, field

import gymnasium as gym
from rcs._core.common import GripperConfig
from rcs.envs.base import (
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    HardwareEnv,
    MultiRobotWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
)
from rcs.envs.scenes import RCSEnvCreator, WrapperConfig
from rcs_yam.hw import Yam, YamConfig, YamGripper

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


@dataclass(kw_only=True)
class YamHardwareEnvCreatorConfig:
    robot_cfg: YamConfig
    control_mode: ControlMode
    gripper_cfg: GripperConfig | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSYamConfigEnvCreator(RCSEnvCreator[YamHardwareEnvCreatorConfig]):
    def create_env(self, cfg: YamHardwareEnvCreatorConfig) -> gym.Env:
        ik = rcs.common.Pin(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = Yam(cfg.robot_cfg, ik)
        env: gym.Env = HardwareEnv()
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)

        if cfg.gripper_cfg is not None:
            # The gripper motor is part of the arm's motor chain, so it shares the robot handle.
            gripper = YamGripper(cfg.gripper_cfg, robot)
            env = GripperWrapper(env, gripper, binary=cfg.wrapper_cfg.binary_gripper)

        if cfg.relative_to != RelativeTo.NONE:
            env = RelativeActionSpace(env, max_mov=cfg.max_relative_movement, relative_to=cfg.relative_to)
        return CoverWrapper(env)

    def config(self) -> YamHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)


@dataclass(kw_only=True)
class YamMultiHardwareEnvCreatorConfig:
    robot_cfgs: dict[str, YamConfig]
    control_mode: ControlMode
    gripper_cfgs: dict[str, GripperConfig | None] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    robot_to_shared_base_frame: dict[str, rcs.common.Pose] | None = None
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSYamMultiConfigEnvCreator(RCSEnvCreator[YamMultiHardwareEnvCreatorConfig]):
    def create_env(self, cfg: YamMultiHardwareEnvCreatorConfig) -> gym.Env:
        envs: dict[str, gym.Env] = {}
        for robot_name, robot_cfg in cfg.robot_cfgs.items():
            envs[robot_name] = RCSYamConfigEnvCreator().create_env(
                YamHardwareEnvCreatorConfig(
                    robot_cfg=robot_cfg,
                    control_mode=cfg.control_mode,
                    gripper_cfg=cfg.gripper_cfgs[robot_name] if cfg.gripper_cfgs is not None else None,
                    max_relative_movement=cfg.max_relative_movement,
                    relative_to=cfg.relative_to,
                    wrapper_cfg=cfg.wrapper_cfg,
                )
            )
        return CoverWrapper(MultiRobotWrapper(envs, cfg.robot_to_shared_base_frame))

    def config(self) -> YamMultiHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
