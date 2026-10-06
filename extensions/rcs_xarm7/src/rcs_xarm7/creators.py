import logging
from dataclasses import dataclass, field
from os import PathLike
from pathlib import Path

import gymnasium as gym
from rcs._core.common import HandConfig
from rcs.camera.hw import HardwareCameraCreatorConfig, create_hardware_camera_set
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    CoverWrapper,
    HandWrapper,
    HardwareEnv,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
)
from rcs.envs.scenes import RCSEnvCreator, WrapperConfig
from rcs_xarm7.hw import XArm7, XArm7Config

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


@dataclass(kw_only=True)
class XArm7HardwareEnvCreatorConfig:
    robot_cfg: XArm7Config
    control_mode: ControlMode
    calibration_dir: PathLike | str | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    hand_cfg: HandConfig | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    frequency: float | None = None
    """Control frequency in Hz, rate limits env.step(). None disables rate limiting."""
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSXArm7ConfigEnvCreator(RCSEnvCreator[XArm7HardwareEnvCreatorConfig]):
    def create_env(self, cfg: XArm7HardwareEnvCreatorConfig) -> gym.Env:
        calibration_dir = cfg.calibration_dir
        if isinstance(calibration_dir, str):
            calibration_dir = Path(calibration_dir)
        ik = rcs.common.Pin(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = XArm7(cfg=cfg.robot_cfg, ik=ik)
        env: gym.Env = HardwareEnv(frequency=cfg.frequency)
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)

        camera_set = create_hardware_camera_set(cfg.camera_cfgs)
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set, include_depth=True)
        if cfg.hand_cfg is not None:
            hand = rcs.registry.HANDS.get(cfg.hand_cfg.hand_type.id)(cfg.hand_cfg)
            env = HandWrapper(env, hand, cfg.wrapper_cfg.binary_gripper)

        if cfg.relative_to != RelativeTo.NONE:
            env = RelativeActionSpace(env, max_mov=cfg.max_relative_movement, relative_to=cfg.relative_to)
        return CoverWrapper(env)

    def config(self) -> XArm7HardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
