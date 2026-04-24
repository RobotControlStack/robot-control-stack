import logging
import typing
from dataclasses import dataclass, field

import gymnasium as gym
from rcs._core.common import BaseCameraConfig, Gripper, GripperConfig
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
from rcs_ur5e.hw import RobotiQGripper, RobotiQGripperConfig, UR5e, UR5eConfig

import rcs

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


def _create_robotiq_gripper(cfg: GripperConfig) -> Gripper:
    if not isinstance(cfg, RobotiQGripperConfig):
        msg = f"Expected RobotiQGripperConfig, got {type(cfg).__name__}"
        raise TypeError(msg)
    return RobotiQGripper(cfg=cfg)


HARDWARE_GRIPPER_CREATORS: dict[str, typing.Callable[[GripperConfig], Gripper]] = {
    "Robotiq2F85siemens": _create_robotiq_gripper,
}


@dataclass(kw_only=True)
class UR5eHardwareEnvCreatorConfig:
    robot_cfg: UR5eConfig
    control_mode: ControlMode
    gripper_cfg: GripperConfig | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSUR5eConfigEnvCreator(RCSEnvCreator[UR5eHardwareEnvCreatorConfig]):
    def create_env(self, cfg: UR5eHardwareEnvCreatorConfig) -> gym.Env:
        ik = rcs.common.Pin(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = UR5e(cfg.robot_cfg, ik)
        env: gym.Env = HardwareEnv()
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)

        if cfg.gripper_cfg is not None:
            gripper_type_id = cfg.gripper_cfg.gripper_type.id
            if gripper_type_id not in HARDWARE_GRIPPER_CREATORS:
                msg = f"Unknown hardware gripper type id: {gripper_type_id}"
                raise ValueError(msg)
            gripper = HARDWARE_GRIPPER_CREATORS[gripper_type_id](cfg.gripper_cfg)
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

    def config(self) -> UR5eHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
