import logging
import typing
from dataclasses import dataclass, field

import gymnasium as gym
import rcs
from rcs._core.common import BaseCameraConfig
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

from rcs_flexiv.hw import Flexiv, FlexivConfig, FlexivGripper, FlexivGripperConfig

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


@dataclass(kw_only=True)
class FlexivHardwareEnvCreatorConfig:
    robot_cfg: FlexivConfig
    control_mode: ControlMode
    gripper_cfg: FlexivGripperConfig | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    frequency: float | None = None
    """Control frequency in Hz, rate limits env.step(). None disables rate limiting."""
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSFlexivConfigEnvCreator(RCSEnvCreator[FlexivHardwareEnvCreatorConfig]):
    def create_env(self, cfg: FlexivHardwareEnvCreatorConfig) -> gym.Env:
        ik = rcs.common.Pin(
            cfg.robot_cfg.kinematic_model_path,
            cfg.robot_cfg.attachment_site,
            urdf=cfg.robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = Flexiv(cfg.robot_cfg, ik)
        env: gym.Env = HardwareEnv(frequency=cfg.frequency)
        env = RobotWrapper(env, robot, cfg.control_mode, home_on_reset=cfg.wrapper_cfg.home_on_reset)

        if cfg.gripper_cfg is not None:
            # The gripper is a device of the same robot connection, so it shares the robot handle.
            gripper = FlexivGripper(cfg.gripper_cfg, robot)
            env = GripperWrapper(env, gripper, binary=cfg.wrapper_cfg.binary_gripper)

        camera_set = _create_hardware_camera_set(cfg.camera_cfgs)
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set, cfg.wrapper_cfg.include_depth)

        if cfg.relative_to != RelativeTo.NONE:
            env = RelativeActionSpace(env, max_mov=cfg.max_relative_movement, relative_to=cfg.relative_to)
        return CoverWrapper(env)

    def config(self) -> FlexivHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)


@dataclass(kw_only=True)
class FlexivMultiHardwareEnvCreatorConfig:
    robot_cfgs: dict[str, FlexivConfig]
    control_mode: ControlMode
    gripper_cfgs: dict[str, FlexivGripperConfig | None] | None = None
    camera_cfgs: dict[str, HardwareCameraCreatorConfig] | None = None
    max_relative_movement: float | tuple[float, float] | None = None
    relative_to: RelativeTo = RelativeTo.LAST_STEP
    robot_to_shared_base_frame: dict[str, rcs.common.Pose] | None = None
    """Pose of each robot's base in the shared base frame, in which actions and observations are expressed."""
    frequency: float | None = None
    """Control frequency in Hz, rate limits env.step(). None disables rate limiting."""
    wrapper_cfg: WrapperConfig = field(default_factory=WrapperConfig)


class RCSFlexivMultiConfigEnvCreator(RCSEnvCreator[FlexivMultiHardwareEnvCreatorConfig]):
    def create_env(self, cfg: FlexivMultiHardwareEnvCreatorConfig) -> gym.Env:
        envs: dict[str, gym.Env] = {}
        for robot_name, robot_cfg in cfg.robot_cfgs.items():
            envs[robot_name] = RCSFlexivConfigEnvCreator().create_env(
                FlexivHardwareEnvCreatorConfig(
                    robot_cfg=robot_cfg,
                    control_mode=cfg.control_mode,
                    gripper_cfg=cfg.gripper_cfgs[robot_name] if cfg.gripper_cfgs is not None else None,
                    # The cameras observe the whole scene, so they are attached once around the
                    # combined env instead of per arm.
                    camera_cfgs=None,
                    max_relative_movement=cfg.max_relative_movement,
                    relative_to=cfg.relative_to,
                    frequency=cfg.frequency,
                    wrapper_cfg=cfg.wrapper_cfg,
                )
            )

        env: gym.Env = MultiRobotWrapper(envs, cfg.robot_to_shared_base_frame)
        camera_set = _create_hardware_camera_set(cfg.camera_cfgs)
        if camera_set is not None:
            camera_set.start()
            camera_set.wait_for_frames()
            logger.info("CameraSet started")
            env = CameraSetWrapper(env, camera_set, cfg.wrapper_cfg.include_depth)
        return CoverWrapper(env)

    def config(self) -> FlexivMultiHardwareEnvCreatorConfig:
        msg = "Implement config() in a subclass or pass `cfg=` explicitly."
        raise NotImplementedError(msg)
