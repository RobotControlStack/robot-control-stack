import logging
from dataclasses import dataclass, field

import gymnasium as gym
from rcs._core.common import GripperConfig
from rcs.camera.hw import HardwareCameraCreatorConfig, create_hardware_camera_set
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    HardwareEnv,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
)
from rcs.envs.scenes import RCSEnvCreator, WrapperConfig
from rcs_ur5e.hw import UR5e, UR5eConfig

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


@dataclass(kw_only=True)
class UR5eHardwareEnvCreatorConfig:
    robot_cfg: UR5eConfig
    control_mode: ControlMode
    gripper_cfg: GripperConfig | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    frequency: float | None = None
    """Control frequency in Hz, rate limits env.step(). None disables rate limiting."""
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSUR5eConfigEnvCreator(RCSEnvCreator[UR5eHardwareEnvCreatorConfig]):
    def create_env(self, cfg: UR5eHardwareEnvCreatorConfig) -> gym.Env:
        ik = rcs.common.Pin(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = UR5e(cfg.robot_cfg, ik)
        env: gym.Env = HardwareEnv(frequency=cfg.frequency)
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)

        if cfg.gripper_cfg is not None:
            gripper = rcs.registry.GRIPPERS.get(cfg.gripper_cfg.gripper_type.id)(cfg.gripper_cfg)
            env = GripperWrapper(env, gripper, binary=cfg.wrapper_cfg.binary_gripper)

        camera_set = create_hardware_camera_set(cfg.camera_cfgs)
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set, include_depth=True)

        if cfg.relative_to != RelativeTo.NONE:
            env = RelativeActionSpace(env, max_mov=cfg.max_relative_movement, relative_to=cfg.relative_to)
        return CoverWrapper(env)

    def config(self) -> UR5eHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
