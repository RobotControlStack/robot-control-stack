import logging
import typing
from dataclasses import dataclass, field

import gymnasium as gym
from rcs._core.common import BaseCameraConfig
from rcs.camera.hw import HardwareCamera, HardwareCameraSet
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
class HardwareCameraCreatorConfig:
    camera_type_id: str
    camera_cfgs: dict[str, BaseCameraConfig]
    kwargs: dict[str, typing.Any] = field(default_factory=dict)


def _create_realsense_camera(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    try:
        from rcs.camera.hw import CalibrationStrategy
        from rcs_realsense.calibration import FR3BaseArucoCalibration
        from rcs_realsense.camera import RealSenseCameraSet
    except ImportError as e:
        msg = "RealSense camera support requires the `rcs_realsense` extension to be installed."
        raise ImportError(msg) from e

    calibration_strategy = {
        name: typing.cast(CalibrationStrategy, FR3BaseArucoCalibration(name)) for name in cfg.camera_cfgs
    }
    return typing.cast(
        HardwareCamera,
        RealSenseCameraSet(cameras=cfg.camera_cfgs, calibration_strategy=calibration_strategy, **cfg.kwargs),
    )


def _create_digit_camera(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    try:
        from rcs.camera.digit_cam import DigitCam
    except ImportError as e:
        msg = "DIGIT camera support requires the `digit_interface` package to be installed."
        raise ImportError(msg) from e

    return typing.cast(HardwareCamera, DigitCam(cameras=cfg.camera_cfgs))


HARDWARE_CAMERA_CREATORS: dict[str, typing.Callable[[HardwareCameraCreatorConfig], HardwareCamera]] = {
    "realsense": _create_realsense_camera,
    "digit": _create_digit_camera,
}


def _create_hardware_camera_set(
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None,
) -> HardwareCameraSet | None:
    if camera_cfgs is None:
        return None
    cameras: list[HardwareCamera] = []
    for cfg in camera_cfgs.values():
        if cfg.camera_type_id not in HARDWARE_CAMERA_CREATORS:
            msg = f"Unknown hardware camera type id: {cfg.camera_type_id}"
            raise ValueError(msg)
        cameras.append(HARDWARE_CAMERA_CREATORS[cfg.camera_type_id](cfg))
    return HardwareCameraSet(cameras) if cameras else None


@dataclass(kw_only=True)
class SO101HardwareEnvCreatorConfig:
    robot_cfg: SO101Config
    control_mode: ControlMode
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSSO101ConfigEnvCreator(RCSEnvCreator[SO101HardwareEnvCreatorConfig]):
    def create_env(self, cfg: SO101HardwareEnvCreatorConfig) -> gym.Env:
        ik = SO101IK(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = SO101(cfg=cfg.robot_cfg, ik=ik)
        env: gym.Env = HardwareEnv()
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)

        gripper = SO101Gripper(robot._hf_robot, robot)
        env = GripperWrapper(env, gripper, binary=cfg.wrapper_cfg.binary_gripper)

        camera_set = _create_hardware_camera_set(cfg.camera_cfgs)
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
