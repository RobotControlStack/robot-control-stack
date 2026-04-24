import logging
import typing
from dataclasses import dataclass, field

import gymnasium as gym
import rcs.hand.tilburg_hand
from rcs._core.common import BaseCameraConfig, Gripper, GripperConfig, GripperType
from rcs.camera.hw import DummyCalibrationStrategy, HardwareCamera, HardwareCameraSet
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
from rcs.hand.tilburg_hand import TilburgHand
from rcs_panda._core import hw
from rcs_panda.envs import PandaHW

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

        # from rcs_realsense.calibration import FR3BaseArucoCalibration
        from rcs_realsense.camera import RealSenseCameraSet
    except ImportError as e:
        msg = "RealSense camera support requires the `rcs_realsense` extension to be installed."
        raise ImportError(msg) from e

    calibration_strategy = {
        name: typing.cast(CalibrationStrategy, DummyCalibrationStrategy()) for name in cfg.camera_cfgs
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


def _create_franka_gripper(cfg: GripperConfig) -> Gripper:
    if not isinstance(cfg, hw.FHConfig):
        msg = f"Expected FHConfig for franka gripper, got {type(cfg).__name__}"
        raise TypeError(msg)
    return hw.FrankaHand(cfg)


def _create_robotiq_gripper(cfg: GripperConfig) -> Gripper:
    try:
        from rcs_robotiq2f85.hw import RobotiQ2F85Gripper, RobotiQ2F85GripperConfig
    except ImportError as e:
        msg = "Robotiq gripper support requires the `rcs_robotiq2f85` extension to be installed."
        raise ImportError(msg) from e

    if not isinstance(cfg, RobotiQ2F85GripperConfig):
        msg = f"Expected RobotiQ2F85GripperConfig, got {type(cfg).__name__}"
        raise TypeError(msg)
    return typing.cast(Gripper, RobotiQ2F85Gripper(cfg))


HARDWARE_GRIPPER_CREATORS: dict[str, typing.Callable[[GripperConfig], Gripper]] = {
    GripperType.FrankaHand.id: _create_franka_gripper,
    "robotiq2f85": _create_robotiq_gripper,
}


@dataclass(kw_only=True)
class PandaHardwareEnvCreatorConfig:
    robot_cfg: hw.PandaConfig
    control_mode: ControlMode
    gripper_cfg: GripperConfig | rcs.hand.tilburg_hand.THConfig | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


@dataclass(kw_only=True)
class PandaMultiHardwareEnvCreatorConfig:
    robot_cfgs: dict[str, hw.PandaConfig]
    control_mode: ControlMode
    gripper_cfgs: dict[str, GripperConfig | rcs.hand.tilburg_hand.THConfig | None] | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    robot_to_shared_base_frame: dict[str, rcs.common.Pose] | None = None
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSPandaConfigEnvCreator(RCSEnvCreator[PandaHardwareEnvCreatorConfig]):
    def create_env(self, cfg: PandaHardwareEnvCreatorConfig) -> gym.Env:
        ik = rcs.common.Pin(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = hw.Franka(cfg.robot_cfg, ik)

        env: gym.Env = HardwareEnv()
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)
        env = PandaHW(env)
        if isinstance(cfg.gripper_cfg, rcs.hand.tilburg_hand.THConfig):
            hand = TilburgHand(cfg.gripper_cfg)
            env = HandWrapper(env, hand, binary=cfg.wrapper_cfg.binary_gripper)
        elif cfg.gripper_cfg is not None:
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
                    camera_cfgs=None,
                    max_relative_movement=cfg.max_relative_movement,
                    relative_to=cfg.relative_to,
                    wrapper_cfg=cfg.wrapper_cfg,
                )
            )

        env: gym.Env = MultiRobotWrapper(envs, cfg.robot_to_shared_base_frame)
        camera_set = _create_hardware_camera_set(cfg.camera_cfgs)
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set)
        return CoverWrapper(env)

    def config(self) -> PandaMultiHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
