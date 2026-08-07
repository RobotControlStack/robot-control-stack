import logging
import typing
from dataclasses import dataclass, field

import gymnasium as gym
from rcs._core.common import BaseCameraConfig, GripperConfig
from rcs.camera.hw import DummyCalibrationStrategy, HardwareCamera, HardwareCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
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
class HardwareCameraCreatorConfig:
    camera_type_id: str
    camera_cfgs: dict[str, BaseCameraConfig]
    kwargs: dict[str, typing.Any] = field(default_factory=dict)


def _create_realsense_camera(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    try:
        from rcs.camera.hw import CalibrationStrategy
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


HARDWARE_CAMERA_CREATORS: dict[str, typing.Callable[[HardwareCameraCreatorConfig], HardwareCamera]] = {
    "realsense": _create_realsense_camera,
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


def _attach_camera_set(
    env: gym.Env, camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None, include_depth: bool
) -> gym.Env:
    camera_set = _create_hardware_camera_set(camera_cfgs)
    if camera_set is None:
        return env
    camera_set.start()
    camera_set.wait_for_frames()
    logger.info("CameraSet started")
    return CameraSetWrapper(env, camera_set, include_depth)


@dataclass(kw_only=True)
class YamHardwareEnvCreatorConfig:
    robot_cfg: YamConfig
    control_mode: ControlMode
    gripper_cfg: GripperConfig | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
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

        env = _attach_camera_set(env, cfg.camera_cfgs, cfg.wrapper_cfg.include_depth)

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
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
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
                    # The cameras observe the whole scene, so they are attached once around the
                    # combined env instead of per arm.
                    camera_cfgs=None,
                    max_relative_movement=cfg.max_relative_movement,
                    relative_to=cfg.relative_to,
                    wrapper_cfg=cfg.wrapper_cfg,
                )
            )

        env: gym.Env = MultiRobotWrapper(envs, cfg.robot_to_shared_base_frame)
        env = _attach_camera_set(env, cfg.camera_cfgs, cfg.wrapper_cfg.include_depth)
        return CoverWrapper(env)

    def config(self) -> YamMultiHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
