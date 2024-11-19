import logging
from os import PathLike

import gymnasium as gym
import rcsss
from rcsss import sim
from rcsss._core.hw import FR3Config, IKController
from rcsss._core.sim import CameraType
from rcsss.camera.hw import BaseHardwareCameraSet
from rcsss.camera.interface import BaseCameraConfig
from rcsss.camera.realsense import RealSenseCameraSet, RealSenseSetConfig
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperWrapper,
    LimitedJointsRelDictType,
    ObsArmsGrCam,
    RelativeActionSpace,
    RelativeTo,
)
from rcsss.envs.hw import FR3HW
from rcsss.envs.sim import CollisionGuard, FR3Sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_fr3_hw_robot_cfg():
    robot_cfg = FR3Config()
    robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    robot_cfg.speed_factor = 0.1
    robot_cfg.controller = IKController.robotics_library
    return robot_cfg


def default_fr3_hw_gripper_cfg():
    gripper_cfg = rcsss.hw.FHConfig()
    gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
    gripper_cfg.speed = 0.1
    gripper_cfg.force = 30
    return gripper_cfg


def default_realsense(name2id: dict[str, str]):
    cameras = {name: BaseCameraConfig(identifier=id) for name, id in name2id.items()}
    cam_cfg = RealSenseSetConfig(
        cameras=cameras,
        resolution_width=1280,
        resolution_height=720,
        frame_rate=15,
        enable_imu=False,  # does not work with imu, why?
        enable_ir=True,
        enable_ir_emitter=False,
    )
    return RealSenseCameraSet(cam_cfg)


def get_urdf_path(urdf_path: str | PathLike | None, allow_none_if_not_found: bool = False) -> str | None:
    if urdf_path is None and "lab" in rcsss.scenes:
        urdf_path = rcsss.scenes["lab"].parent / "fr3.urdf"
        assert urdf_path.exists(), "Automatic deduced urdf path does not exist. Corrupted models directory."
        logger.info("Using automatic found urdf.")
    elif urdf_path is None and not allow_none_if_not_found:
        msg = "This pip package was not built with the UTN lab models, please pass the urdf and mjcf path to use simulation or collision guard."
        raise ValueError(msg)
    elif urdf_path is None:
        logger.warning("No urdf path was found. Proceeding, but set_cartesian methods will result in errors.")
    return str(urdf_path) if urdf_path is not None else None


def fr3_hw_env(
    ip: str,
    control_mode: ControlMode,
    robot_cfg: rcsss.hw.FR3Config,
    collision_guard: str | PathLike | None = None,
    gripper_cfg: rcsss.hw.FHConfig | None = None,
    camera_set: BaseHardwareCameraSet | None = None,
    max_relative_movement: float | tuple[float, float] | None = None,
    relative_to: RelativeTo = RelativeTo.LAST_STEP,
    urdf_path: str | PathLike | None = None,
) -> gym.Env:
    """
    Creates a hardware environment for the FR3 robot.

    Args:
        ip (str): IP address of the robot.
        control_mode (ControlMode): Control mode for the robot.
        robot_cfg (rcsss.hw.FR3Config): Configuration for the FR3 robot.
        collision_guard (str | PathLike | None): Key to a scene (requires UTN compatible scene package to be present)
            or the path to a mujoco scene for collision guarding. If None, collision guarding is not used.
        gripper_cfg (rcsss.hw.FHConfig | None): Configuration for the gripper. If None, no gripper is used.
        camera_set (BaseHardwareCameraSet | None): Camera set to be used. If None, no cameras are used.
        max_relative_movement (float | tuple[float, float] | None): Maximum allowed movement. If float, it restricts
            translational movement in meters. If tuple, it restricts both translational (in meters) and rotational
            (in radians) movements. If None, no restriction is applied.
        relative_to (RelativeTo): Specifies whether the movement is relative to a configured origin or the last step.
        urdf_path (str | PathLike | None): Path to the URDF file. If None, the URDF file is automatically deduced
            which requires a UTN compatible lab scene to be present. If no URDF file is found, the environment will
            still work but set_cartesian methods might throw an error. A URDF file is needed for collision guarding.

    Returns:
        gym.Env: The configured hardware environment for the FR3 robot.
    """
    urdf_path = get_urdf_path(urdf_path, allow_none_if_not_found=collision_guard is not None)
    ik = rcsss.common.IK(str(urdf_path)) if urdf_path is not None else None
    robot = rcsss.hw.FR3(ip, ik)
    robot.set_parameters(robot_cfg)

    env: gym.Env = FR3Env(robot, ControlMode.JOINTS if collision_guard is not None else control_mode)

    env = FR3HW(env)
    if gripper_cfg is not None:
        gripper = rcsss.hw.FrankaHand(ip, gripper_cfg)
        env = GripperWrapper(env, gripper, binary=True)

    if camera_set is not None:
        camera_set.start()
        camera_set.wait_for_frames()
        logger.info("CameraSet started")
        env = CameraSetWrapper(env, camera_set)

    if collision_guard is not None:
        assert urdf_path is not None
        env = CollisionGuard.env_from_xml_paths(
            env,
            str(rcsss.scenes.get(str(collision_guard), collision_guard)),
            urdf_path,
            gripper=True,
            check_home_collision=False,
            camera=True,
            control_mode=control_mode,
            tcp_offset=rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset()),
        )
    if max_relative_movement is not None:
        env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

    return env


def default_fr3_sim_robot_cfg():
    cfg = sim.FR3Config()
    cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    cfg.realtime = False
    return cfg


def default_fr3_sim_gripper_cfg():
    return sim.FHConfig()


def default_mujoco_cameraset_cfg():

    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed)),
        # "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free)),
        "openvla_view": SimCameraConfig(identifier="openvla_view", type=int(CameraType.fixed)),

        "side": SimCameraConfig(identifier="side_view", type=int(CameraType.fixed)),
        "right_side": SimCameraConfig(identifier="right_side", type=int(CameraType.fixed)),
        "left_side": SimCameraConfig(identifier="left_side", type=int(CameraType.fixed)),
        "front": SimCameraConfig(identifier="front", type=int(CameraType.fixed)),
        "bird_eye": SimCameraConfig(identifier="bird-eye-cam", type=int(CameraType.fixed)),
    }
    # 256x256 needed for VLAs
    return SimCameraSetConfig(
        cameras=cameras, resolution_width=256, resolution_height=256, frame_rate=10, physical_units=True
    )


def fr3_sim_env(
    control_mode: ControlMode,
    robot_cfg: rcsss.sim.FR3Config,
    collision_guard: str | PathLike | None = None,
    gripper_cfg: rcsss.sim.FHConfig | None = None,
    camera_set_cfg: SimCameraSetConfig | None = None,
    max_relative_movement: float | tuple[float, float] | None = None,
    relative_to: RelativeTo = RelativeTo.LAST_STEP,
    urdf_path: str | PathLike | None = None,
    mjcf: str | PathLike = "fr3_empty_world",
) -> gym.Env[ObsArmsGrCam, LimitedJointsRelDictType]:
    """
    Creates a simulation environment for the FR3 robot.

    Args:
        control_mode (ControlMode): Control mode for the robot.
        robot_cfg (rcsss.sim.FR3Config): Configuration for the FR3 robot.
        collision_guard (str | PathLike | None): Key to a scene (requires UTN compatible scene package to be present)
            or the path to a mujoco scene for collision guarding. If None, collision guarding is not used.
        gripper_cfg (rcsss.sim.FHConfig | None): Configuration for the gripper. If None, no gripper is used.
        camera_set_cfg (SimCameraSetConfig | None): Configuration for the camera set. If None, no cameras are used.
        max_relative_movement (float | tuple[float, float] | None): Maximum allowed movement. If float, it restricts
            translational movement in meters. If tuple, it restricts both translational (in meters) and rotational
            (in radians) movements. If None, no restriction is applied.
        relative_to (RelativeTo): Specifies whether the movement is relative to a configured origin or the last step.
        urdf_path (str | PathLike | None): Path to the URDF file. If None, the URDF file is automatically deduced
            which requires a UTN compatible lab scene to be present.
        mjcf (str | PathLike): Path to the Mujoco scene XML file. Defaults to "fr3_empty_world".

    Returns:
        gym.Env: The configured simulation environment for the FR3 robot.
    """
    urdf_path = get_urdf_path(urdf_path, allow_none_if_not_found=False)
    assert urdf_path is not None
    if mjcf not in rcsss.scenes:
        logger.warning("mjcf not found as key in scenes, interpreting mjcf as path the mujoco scene xml")

    simulation = sim.Sim(rcsss.scenes.get(str(mjcf), mjcf))
    ik = rcsss.common.IK(urdf_path)
    robot = rcsss.sim.FR3(simulation, "0", ik)
    robot.set_parameters(robot_cfg)
    env: gym.Env = FR3Env(robot, control_mode)
    env = FR3Sim(env, simulation)

    if camera_set_cfg is not None:
        camera_set = SimCameraSet(simulation, camera_set_cfg)
        env = CameraSetWrapper(env, camera_set, include_depth=True)

    if gripper_cfg is not None:
        gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
        env = GripperWrapper(env, gripper, binary=True)

    if collision_guard is not None:
        env = CollisionGuard.env_from_xml_paths(
            env,
            str(rcsss.scenes.get(str(collision_guard), collision_guard)),
            urdf_path,
            gripper=gripper_cfg is not None,
            check_home_collision=False,
            camera=False,
            control_mode=control_mode,
            tcp_offset=rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset()),
        )
    if max_relative_movement is not None:
        env = RelativeActionSpace(env, max_mov=max_relative_movement, relative_to=relative_to)

    return env
