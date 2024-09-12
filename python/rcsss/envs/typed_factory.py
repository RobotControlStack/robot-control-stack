import typing

import gymnasium as gym
from rcsss import hw, sim
from rcsss._core.hw import FHConfig
from rcsss._core.sim import CameraType
from rcsss.camera.realsense import RealSenseCameraSet, RealSenseSetConfig
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.config import read_config_yaml
from rcsss.desk import Desk
from rcsss.envs.base import (
    ArmObsType,
    CameraSetWrapper,
    CartOrJointContType,
    ControlMode,
    FR3Env,
    GripperWrapper,
    JointsDictType,
    LimitedJointsRelDictType,
    LimitedTQuartRelDictType,
    LimitedTRPYRelDictType,
    ObsArmsGr,
    ObsArmsGrCam,
    RelativeActionSpace,
    TQuartDictType,
    TRPYDictType,
)
from rcsss.envs.hw import FR3HW
from rcsss.envs.sim import CollisionGuard, FR3Sim


def produce_env_sim(
    mjcf_path: str, urdf_path: str, control_mode: ControlMode, cfg: sim.FR3Config, robot_id: str = "0"
) -> tuple[gym.Env[ArmObsType, CartOrJointContType], dict[str, typing.Any]]:
    simulation = sim.Sim(mjcf_path)
    robot = sim.FR3(simulation, robot_id, urdf_path)
    robot.set_parameters(cfg)
    env = FR3Env(robot, control_mode)
    env_sim = FR3Sim(env, simulation)
    return typing.cast(gym.Env[ArmObsType, CartOrJointContType], FR3Sim(env_sim, simulation)), {
        "sim": simulation,
        "robot": robot,
    }


def produce_env_sim_joints(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id: str = "0"
) -> tuple[gym.Env[ArmObsType, JointsDictType], dict[str, typing.Any]]:
    env, info = produce_env_sim(mjcf_path, urdf_path, control_mode=ControlMode.JOINTS, cfg=cfg, robot_id=robot_id)
    return (
        typing.cast(
            gym.Env[ArmObsType, JointsDictType],
            env,
        ),
        info,
    )


def produce_env_sim_trpy(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id: str = "0"
) -> tuple[gym.Env[ArmObsType, TRPYDictType], dict[str, typing.Any]]:
    env, info = produce_env_sim(
        mjcf_path, urdf_path, control_mode=ControlMode.CARTESIAN_TRPY, cfg=cfg, robot_id=robot_id
    )
    return (
        typing.cast(
            gym.Env[ArmObsType, TRPYDictType],
            env,
        ),
        info,
    )


def produce_env_sim_tquart(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id: str = "0"
) -> tuple[gym.Env[ArmObsType, TQuartDictType], dict[str, typing.Any]]:
    env, info = produce_env_sim(
        mjcf_path, urdf_path, control_mode=ControlMode.CARTESIAN_TQuart, cfg=cfg, robot_id=robot_id
    )
    return typing.cast(gym.Env[ArmObsType, TQuartDictType], env), info


def produce_env_sim_tquart_gripper_camera(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, TQuartDictType]:
    """Environment with tquart, gripper, camera, collision guard and relative limits"""
    env_sim, info = produce_env_sim_tquart(mjcf_path, urdf_path, cfg, robot_id)
    simulation = info["sim"]
    gripper = sim.FrankaHand(simulation, robot_id, gripper_cfg)
    env_sim_gripper = GripperWrapper(env_sim, gripper)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_sim_gripper_cam: gym.Env = CameraSetWrapper(env_sim_gripper, camera_set)
    return typing.cast(
        gym.Env[ObsArmsGrCam, TQuartDictType],
        env_sim_gripper_cam,
    )


def produce_env_sim_tquart_gripper_camera_rel(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType]:
    """Environment with tquart, gripper, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_tquart_gripper_camera(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_rel = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType],
        env_rel,
    )


def produce_env_sim_tquart_gripper_camera_cg(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, TQuartDictType]:
    """Environment with tquart, gripper, camera, collision guard and relative limits"""
    env_sim_tquart_gripper_cam = produce_env_sim_tquart_gripper_camera(
        mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id
    )
    env_sim_tquart_gripper_cam_cg = CollisionGuard.env_from_xml_paths(
        env_sim_tquart_gripper_cam, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(
        gym.Env[ObsArmsGrCam, TQuartDictType],
        env_sim_tquart_gripper_cam_cg,
    )


def produce_env_sim_tquart_gripper_camera_cg_rel(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType]:
    """Environment with tquart, gripper, camera, collision guard and relative limits"""
    env_sim_tquart_gripper_cam = produce_env_sim_tquart_gripper_camera_cg(
        mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id
    )
    env_sim_tquart_gripper_cam_cg = RelativeActionSpace(env_sim_tquart_gripper_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType],
        env_sim_tquart_gripper_cam_cg,
    )


def produce_env_sim_trpy_gripper_camera(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, TRPYDictType]:
    """Environment with trpy, gripper, camera, collision guard and relative limits"""
    env_sim_trpy, info = produce_env_sim_trpy(mjcf_path, urdf_path, cfg, robot_id)
    simulation = info["sim"]
    gripper = sim.FrankaHand(simulation, robot_id, gripper_cfg)
    env_sim_trpy_gripper = GripperWrapper(env_sim_trpy, gripper)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_sim_trpy_gripper_cam: gym.Env = CameraSetWrapper(env_sim_trpy_gripper, camera_set)
    return typing.cast(
        gym.Env[ObsArmsGrCam, TRPYDictType],
        env_sim_trpy_gripper_cam,
    )


def produce_env_sim_trpy_gripper_camera_rel(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, TRPYDictType]:
    """Environment with trpy, gripper, camera, collision guard and relative limits"""
    env_sim_trpy_gripper_cam = produce_env_sim_trpy_gripper_camera(
        mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id
    )
    env_sim_trpy_gripper_cam_rel = RelativeActionSpace(env_sim_trpy_gripper_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType],
        env_sim_trpy_gripper_cam_rel,
    )


def produce_env_sim_trpy_gripper_camera_cg(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, TRPYDictType]:
    """Environment with trpy, gripper, camera, collision guard and relative limits"""
    env_sim_trpy_gripper_cam = produce_env_sim_trpy_gripper_camera(
        mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id
    )
    env_sim_trpy_gripper_cam_cg = CollisionGuard.env_from_xml_paths(
        env_sim_trpy_gripper_cam, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(
        gym.Env[ObsArmsGrCam, TRPYDictType],
        env_sim_trpy_gripper_cam_cg,
    )


def produce_env_sim_trpy_gripper_camera_cg_rel(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType]:
    """Environment with trpy, gripper, camera, collision guard and relative limits"""
    env_sim_trpy_gripper_camera_cg = produce_env_sim_trpy_gripper_camera_cg(
        mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id
    )
    env_sim_trpy_gripper_camera_cg_rel = RelativeActionSpace(env_sim_trpy_gripper_camera_cg)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType],
        env_sim_trpy_gripper_camera_cg_rel,
    )


def produce_env_sim_joints_gripper_camera(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, JointsDictType]:
    """Environment with joints, gripper, camera"""
    env_sim_joints, info = produce_env_sim_joints(mjcf_path, urdf_path, cfg, robot_id)
    simulation = info["sim"]
    gripper = sim.FrankaHand(simulation, robot_id, gripper_cfg)
    env_sim_joints_gripper = GripperWrapper(env_sim_joints, gripper)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_sim_joints_gripper_cam: gym.Env = CameraSetWrapper(env_sim_joints_gripper, camera_set)
    return typing.cast(
        gym.Env[ObsArmsGrCam, JointsDictType],
        env_sim_joints_gripper_cam,
    )


def produce_env_sim_joints_gripper_camera_rel(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, JointsDictType]:
    """Environment with joints, gripper, camera, collision guard and relative limits"""
    env_sim_joints_cam = produce_env_sim_joints_gripper_camera(
        mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id
    )
    env_sim_joints_cam_rel = RelativeActionSpace(env_sim_joints_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedJointsRelDictType],
        env_sim_joints_cam_rel,
    )


def produce_env_sim_joints_gripper_camera_cg(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, JointsDictType]:
    """Environment with joints, gripper, camera, collision guard and relative limits"""
    env_sim_joints_gripper_cam = produce_env_sim_joints_gripper_camera(
        mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id
    )
    env_sim_joints_gripper_cam_cg = CollisionGuard.env_from_xml_paths(
        env_sim_joints_gripper_cam, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(
        gym.Env[ObsArmsGrCam, JointsDictType],
        env_sim_joints_gripper_cam_cg,
    )


def produce_env_sim_joints_gripper_camera_cg_rel(
    mjcf_path: str,
    urdf_path: str,
    cfg: sim.FR3Config,
    gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig,
    robot_id: str = "0",
) -> gym.Env[ObsArmsGrCam, LimitedJointsRelDictType]:
    """Environment with joints, gripper, camera, collision guard and relative limits"""
    env_sim_joints_gripper_cam_cg = produce_env_sim_joints_gripper_camera_cg(
        mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id
    )
    env_sim_joints_gripper_cam_cg_rel = RelativeActionSpace(env_sim_joints_gripper_cam_cg)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedJointsRelDictType],
        env_sim_joints_gripper_cam_cg_rel,
    )


def produce_env_hw(
    ip: str, urdf_path: str, control_mode: ControlMode, cfg_path: str
) -> gym.Env[ArmObsType, CartOrJointContType]:
    cfg = read_config_yaml(cfg_path)
    with Desk(ip, cfg.hw.username, cfg.hw.password) as d:
        d.unlock()
        d.activate_fci()
        robot = hw.FR3(ip, urdf_path)
        env = FR3Env(robot, control_mode)
        env_hw = FR3HW(env)

    return typing.cast(gym.Env[ArmObsType, CartOrJointContType], env_hw)


def produce_env_hw_joints(ip: str, urdf_path: str, cfg_path: str) -> gym.Env[ArmObsType, JointsDictType]:
    env_hw_joints = produce_env_hw(ip, urdf_path, ControlMode.JOINTS, cfg_path)
    return typing.cast(gym.Env[ArmObsType, JointsDictType], env_hw_joints)


def produce_env_hw_joints_gripper(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cfg_path: str
) -> (gym.Env)[ObsArmsGr, JointsDictType]:
    env_hw_joints = produce_env_hw_joints(ip, urdf_path, cfg_path)
    gripper = hw.FrankaHand(ip, gripper_cfg)
    env_hw_joints_gripper = GripperWrapper(env_hw_joints, gripper)
    return typing.cast(gym.Env[ObsArmsGr, JointsDictType], env_hw_joints_gripper)


def produce_env_hw_joints_gripper_cg(
    ip: str, urdf_path: str, mjcf_path: str, gripper_cfg: FHConfig, cfg_path: str
) -> gym.Env[ObsArmsGr, JointsDictType]:
    env_hw_joints_gripper = produce_env_hw_joints_gripper(ip, urdf_path, gripper_cfg, cfg_path)
    env_hw_joints_gripper_cg = CollisionGuard.env_from_xml_paths(
        env_hw_joints_gripper, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(
        gym.Env[ObsArmsGr, JointsDictType],
        env_hw_joints_gripper_cg,
    )


def produce_env_hw_joints_gripper_cg_rel(
    ip: str, urdf_path: str, mjcf_path: str, gripper_cfg: FHConfig, cfg_path: str
) -> gym.Env[ObsArmsGr, LimitedJointsRelDictType]:
    env_hw_joints_gripper_cg = produce_env_hw_joints_gripper_cg(ip, urdf_path, mjcf_path, gripper_cfg, cfg_path)
    env_hw_joints_gripper_cg_rel = RelativeActionSpace(env_hw_joints_gripper_cg)
    return typing.cast(gym.Env[ObsArmsGr, LimitedJointsRelDictType], env_hw_joints_gripper_cg_rel)


def produce_env_hw_joints_gripper_camera(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> (gym.Env)[ObsArmsGrCam, JointsDictType]:
    env_hw_joints_gripper = produce_env_hw_joints_gripper(ip, urdf_path, gripper_cfg, cfg_path)
    camera_set = RealSenseCameraSet(cam_cfg)
    env_hw_joints_gripper_cam: gym.Env = CameraSetWrapper(env_hw_joints_gripper, camera_set)
    return typing.cast(gym.Env[ObsArmsGrCam, JointsDictType], env_hw_joints_gripper_cam)


def produce_env_hw_joints_gripper_camera_rel(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, LimitedJointsRelDictType]:
    env_hw_joints_gripper_cam = produce_env_hw_joints_gripper_camera(ip, urdf_path, gripper_cfg, cam_cfg, cfg_path)
    env_sim_joints_gripper_cam_rel = RelativeActionSpace(env_hw_joints_gripper_cam)
    return typing.cast(gym.Env[ObsArmsGrCam, LimitedJointsRelDictType], env_sim_joints_gripper_cam_rel)


def produce_env_hw_joints_gripper_camera_cg(
    ip: str, urdf_path: str, mjcf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, JointsDictType]:
    env_hw_joints_gripper_cam = produce_env_hw_joints_gripper_camera(ip, urdf_path, gripper_cfg, cam_cfg, cfg_path)
    env_hw_joints_gripper_cam_cg = CollisionGuard.env_from_xml_paths(
        env_hw_joints_gripper_cam, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(gym.Env[ObsArmsGrCam, JointsDictType], env_hw_joints_gripper_cam_cg)


def produce_env_hw_joints_gripper_camera_cg_rel(
    ip: str, urdf_path: str, mjcf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, LimitedJointsRelDictType]:
    env_hw_joints_gripper_cam_cg = produce_env_hw_joints_gripper_camera_cg(
        ip, urdf_path, mjcf_path, gripper_cfg, cam_cfg, cfg_path
    )
    env_hw_joints_gripper_cam_cg_rel = RelativeActionSpace(env_hw_joints_gripper_cam_cg)
    return typing.cast(gym.Env[ObsArmsGrCam, LimitedJointsRelDictType], env_hw_joints_gripper_cam_cg_rel)


def produce_env_hw_trpy(ip: str, urdf_path: str, cfg_path: str) -> gym.Env[ArmObsType, TRPYDictType]:
    env_hw_trpy = produce_env_hw(ip, urdf_path, ControlMode.CARTESIAN_TRPY, cfg_path)
    return typing.cast(gym.Env[ArmObsType, TRPYDictType], env_hw_trpy)


def produce_env_hw_trpy_gripper(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cfg_path: str
) -> (gym.Env)[ObsArmsGr, TRPYDictType]:
    env_hw_joints = produce_env_hw_trpy(ip, urdf_path, cfg_path)
    gripper = hw.FrankaHand(ip, gripper_cfg)
    env_hw_trpy_gripper = GripperWrapper(env_hw_joints, gripper)
    return typing.cast(gym.Env[ObsArmsGr, TRPYDictType], env_hw_trpy_gripper)


def produce_env_hw_trpy_gripper_camera(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, TRPYDictType]:
    env_hw_trpy_gripper = produce_env_hw_trpy_gripper(ip, urdf_path, gripper_cfg, cfg_path)
    camera_set = RealSenseCameraSet(cam_cfg)
    env_hw_trpy_gripper_cam: gym.Env = CameraSetWrapper(env_hw_trpy_gripper, camera_set)
    return typing.cast(gym.Env[ObsArmsGrCam, TRPYDictType], env_hw_trpy_gripper_cam)


def produce_env_hw_trpy_gripper_camera_rel(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType]:
    env_hw_trpy_gripper_cam = produce_env_hw_trpy_gripper_camera(ip, urdf_path, gripper_cfg, cam_cfg, cfg_path)
    env_hw_trpy_gripper_cam_rel = RelativeActionSpace(env_hw_trpy_gripper_cam)
    return typing.cast(gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType], env_hw_trpy_gripper_cam_rel)


def produce_env_hw_trpy_gripper_camera_cg(
    ip: str, urdf_path: str, mjcf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, TRPYDictType]:
    env_hw_trpy_gripper_cam = produce_env_hw_trpy_gripper_camera(ip, urdf_path, gripper_cfg, cam_cfg, cfg_path)
    env_hw_trpy_gripper_cam_cg = CollisionGuard.env_from_xml_paths(
        env_hw_trpy_gripper_cam, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(gym.Env[ObsArmsGrCam, TRPYDictType], env_hw_trpy_gripper_cam_cg)


def produce_env_hw_trpy_gripper_camera_cg_rel(
    ip: str, urdf_path: str, mjcf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType]:
    env_hw_trpy_gripper_cam = produce_env_hw_trpy_gripper_camera(ip, urdf_path, gripper_cfg, cam_cfg, cfg_path)
    env_hw_joints_gripper_cam_cg = CollisionGuard.env_from_xml_paths(
        env_hw_trpy_gripper_cam, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType], env_hw_joints_gripper_cam_cg)


def produce_env_hw_tquart(ip: str, urdf_path: str, cfg_path: str) -> gym.Env[ArmObsType, TQuartDictType]:
    env_hw_tquart = produce_env_hw(ip, urdf_path, ControlMode.CARTESIAN_TQuart, cfg_path)
    return typing.cast(gym.Env[ArmObsType, TQuartDictType], env_hw_tquart)


def produce_env_hw_tquart_gripper(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cfg_path: str
) -> (gym.Env)[ObsArmsGr, TQuartDictType]:
    env_hw_tquart = produce_env_hw_tquart(ip, urdf_path, cfg_path)
    gripper = hw.FrankaHand(ip, gripper_cfg)
    env_hw_tquart_gripper = GripperWrapper(env_hw_tquart, gripper)
    return typing.cast(gym.Env[ObsArmsGr, TQuartDictType], env_hw_tquart_gripper)


def produce_env_hw_tquart_gripper_camera(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, TQuartDictType]:
    env_hw_tquart_gripper = produce_env_hw_tquart_gripper(ip, urdf_path, gripper_cfg, cfg_path)
    camera_set = RealSenseCameraSet(cam_cfg)
    env_hw_tquart_gripper_cam: gym.Env = CameraSetWrapper(env_hw_tquart_gripper, camera_set)
    return typing.cast(gym.Env[ObsArmsGrCam, TQuartDictType], env_hw_tquart_gripper_cam)


def produce_env_hw_tquart_gripper_camera_rel(
    ip: str, urdf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType]:
    env_hw_tquart_gripper_cam = produce_env_hw_tquart_gripper_camera(ip, urdf_path, gripper_cfg, cam_cfg, cfg_path)
    env_sim_tquart_gripper_cam_rel = RelativeActionSpace(env_hw_tquart_gripper_cam)
    return typing.cast(gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType], env_sim_tquart_gripper_cam_rel)


def produce_env_hw_tquart_gripper_camera_cg(
    ip: str, urdf_path: str, mjcf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, TQuartDictType]:
    env_hw_tquart_gripper_cam = produce_env_hw_tquart_gripper_camera(ip, urdf_path, gripper_cfg, cam_cfg, cfg_path)
    env_hw_tquart_gripper_cam_cg = CollisionGuard.env_from_xml_paths(
        env_hw_tquart_gripper_cam, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(gym.Env[ObsArmsGrCam, TQuartDictType], env_hw_tquart_gripper_cam_cg)


def produce_env_hw_tquart_gripper_camera_cg_rel(
    ip: str, urdf_path: str, mjcf_path: str, gripper_cfg: FHConfig, cam_cfg: RealSenseSetConfig, cfg_path: str
) -> gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType]:
    env_hw_tquart_gripper_cam = produce_env_hw_tquart_gripper_camera_cg(
        ip, urdf_path, mjcf_path, gripper_cfg, cam_cfg, cfg_path
    )
    env_hw_tquart_gripper_cam_cg = CollisionGuard.env_from_xml_paths(
        env_hw_tquart_gripper_cam, mjcf_path, urdf_path, gripper=True, check_home_collision=False, camera=True
    )
    return typing.cast(gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType], env_hw_tquart_gripper_cam_cg)


if __name__ == "__main__":
    mjcf = "models/mjcf/fr3_modular/scene.xml"
    urdf = "models/fr3/urdf/fr3_from_panda.urdf"
    robot_id_ = "0"

    cfg_ = sim.FR3Config()
    cfg_.ik_duration_in_milliseconds = 300
    cfg_.realtime = False
    gripper_cfg = sim.FHConfig()

    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
    }
    cam_cfg_ = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=50)

    env_ = produce_env_sim_tquart_gripper_camera_rel(
        mjcf_path=mjcf, urdf_path=urdf, cfg=cfg_, gripper_cfg=gripper_cfg, cam_cfg=cam_cfg_, robot_id=robot_id_
    )
    obs_, info_ = env_.reset()
    for _ in range(100):
        act = env_.action_space.sample()
        act["tquart"][3:] = [0, 0, 0, 1]
        obs, reward, terminated, truncated, info = env_.step(act)
        if truncated or terminated:
            env_.reset()
