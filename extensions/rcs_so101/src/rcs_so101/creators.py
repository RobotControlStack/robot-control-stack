import logging
from dataclasses import dataclass, field

import gymnasium as gym
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
from rcs_so101._core.so101_ik import SO101IK
from rcs_so101.hw import SO101, SO101Config, SO101Gripper

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


@dataclass(kw_only=True)
class SO101HardwareEnvCreatorConfig:
    robot_cfg: SO101Config
    control_mode: ControlMode
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    frequency: float | None = None
    """Control frequency in Hz, rate limits env.step(). None disables rate limiting."""
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSSO101ConfigEnvCreator(RCSEnvCreator[SO101HardwareEnvCreatorConfig]):
    def create_env(self, cfg: SO101HardwareEnvCreatorConfig) -> gym.Env:
        ik = SO101IK(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = SO101(cfg=cfg.robot_cfg, ik=ik)
        env: gym.Env = HardwareEnv(frequency=cfg.frequency)
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)

        gripper = SO101Gripper(robot._hf_robot, robot)
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

    def config(self) -> SO101HardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)

    # For now, the leader-follower teleop script uses the leader object directly
    # and doesn't depend on an RCS-provided class.
    # @staticmethod
    # def teleoperator(
    #     id: str,
    #     port: str,
    #     calibration_dir: PathLike | str | None = None,
    # ) -> SO101Leader:
    #     if isinstance(calibration_dir, str):
    #         calibration_dir = Path(calibration_dir)
    #     cfg = SO101LeaderConfig(id=id, calibration_dir=calibration_dir, port=port)
    #     teleop = make_teleoperator_from_config(cfg)
    #     teleop.connect()
    #     return teleop
