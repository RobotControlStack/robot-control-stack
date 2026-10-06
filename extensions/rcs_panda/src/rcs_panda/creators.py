import logging
from dataclasses import dataclass, field

import gymnasium as gym
from rcs._core.common import Gripper, GripperConfig, HandConfig
from rcs.camera.hw import HardwareCameraCreatorConfig, create_hardware_camera_set
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    HandWrapper,
    HardwareEnv,
    MultiRobotWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
)
from rcs.envs.scenes import RCSEnvCreator, WrapperConfig
from rcs_panda._core import hw
from rcs_panda.envs import PandaHW

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def create_panda_gripper(cfg: GripperConfig) -> Gripper:
    # The Panda hand is the Franka hand driven by this extension's libfranka build, so it has its
    # own type id to keep it apart from `rcs_fr3`'s `FrankaHand` when both are installed.
    if not isinstance(cfg, hw.FHConfig):
        msg = f"Expected rcs_panda FHConfig for the panda hand, got {type(cfg).__module__}.{type(cfg).__qualname__}"
        raise TypeError(msg)
    return hw.FrankaHand(cfg)


@dataclass(kw_only=True)
class PandaHardwareEnvCreatorConfig:
    robot_cfg: hw.PandaConfig
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
class PandaMultiHardwareEnvCreatorConfig:
    robot_cfgs: dict[str, hw.PandaConfig]
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


class RCSPandaConfigEnvCreator(RCSEnvCreator[PandaHardwareEnvCreatorConfig]):
    def create_env(self, cfg: PandaHardwareEnvCreatorConfig) -> gym.Env:
        ik = rcs.common.Pin(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = hw.Franka(cfg.robot_cfg, ik)

        env: gym.Env = HardwareEnv(frequency=cfg.frequency)
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)
        env = PandaHW(env)
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
            env = CameraSetWrapper(env, camera_set)

        if cfg.relative_to != RelativeTo.NONE:
            env = RelativeActionSpace(env, max_mov=cfg.max_relative_movement, relative_to=cfg.relative_to)
        return CoverWrapper(env)

    def config(self) -> PandaHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)


class RCSPandaMultiConfigEnvCreator(RCSEnvCreator[PandaMultiHardwareEnvCreatorConfig]):
    def create_env(self, cfg: PandaMultiHardwareEnvCreatorConfig) -> gym.Env:
        envs: dict[str, gym.Env] = {}
        for robot_name, robot_cfg in cfg.robot_cfgs.items():
            envs[robot_name] = RCSPandaConfigEnvCreator().create_env(
                PandaHardwareEnvCreatorConfig(
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
            env = CameraSetWrapper(env, camera_set)
        return CoverWrapper(env)

    def config(self) -> PandaMultiHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
