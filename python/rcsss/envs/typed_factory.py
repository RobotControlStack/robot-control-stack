import typing
import rcsss
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.envs.hw import FR3HW
from rcsss.envs.sim import CollisionGuard
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperWrapper,
    RelativeActionSpace,
    ObsArmsGrCam,
    LimitedCartOrJointContType,
    LimitedTQuartRelDictType,
    LimitedTRPYRelDictType,
    LimitedJointsRelDictType
)
from rcsss import sim
from rcsss.envs.base import ArmObsType, CameraSetWrapper, CartOrJointContType, ControlMode, FR3Env, JointsDictType, TQuartDictType, TRPYDictType
from rcsss.envs.sim import FR3Sim
import gymnasium as gym
from rcsss import hw


def produce_env_sim(
    mjcf_path: str, urdf_path: str, control_mode: ControlMode, cfg: sim.FR3Config, robot_id: str = "0"
) -> tuple[gym.Env[ArmObsType, CartOrJointContType], dict[str, typing.Any]]:
    simulation = sim.Sim(mjcf_path)
    robot = sim.FR3(simulation, robot_id, urdf_path)
    robot.set_parameters(cfg)
    env = FR3Env(robot, control_mode)
    env_sim = FR3Sim(env, simulation)
    return typing.cast(gym.Env[ArmObsType, CartOrJointContType], FR3Sim(env_sim, simulation)), {"sim": simulation,
                                                                                                "robot": robot}


def produce_env_sim_joints(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id: str = "0"
) -> tuple[gym.Env[ArmObsType, JointsDictType], dict[str, typing.Any]]:
    env, info = produce_env_sim(mjcf_path, urdf_path, control_mode=ControlMode.JOINTS, cfg=cfg, robot_id=robot_id)
    return typing.cast(
        gym.Env[ArmObsType, JointsDictType],
        env,
    ), info


def produce_env_sim_trpy(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id: str = "0"
) -> tuple[gym.Env[ArmObsType, TRPYDictType], dict[str, typing.Any]]:
    env, info = produce_env_sim(mjcf_path, urdf_path, control_mode=ControlMode.CARTESIAN_TRPY, cfg=cfg, robot_id=robot_id)
    return typing.cast(
        gym.Env[ArmObsType, TRPYDictType],
        env,
    ), info


def produce_env_sim_tquart(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id: str = "0"
) -> tuple[gym.Env[ArmObsType, TQuartDictType], dict[str, typing.Any]]:
    env, info = produce_env_sim(mjcf_path, urdf_path, control_mode=ControlMode.CARTESIAN_TQuart, cfg=cfg, robot_id=robot_id)
    return typing.cast(gym.Env[ArmObsType, TQuartDictType], env), info


def produce_env_sim_tquart_gripper_camera(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig, robot_id: str = "0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
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
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, LimitedTQuartRelDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_tquart_gripper_camera(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_rel = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType],
        env_rel,
    )

# --- fixed until here ----

def produce_env_sim_tquart_gripper_camera_cg(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_sim = produce_env_sim_tquart_gripper_camera(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_cam = CollisionGuard.env_from_xml_paths(
        env_sim,
        mjcf_path,
        urdf_path,
        gripper=True,
        check_home_collision=False,
    )
    return typing.cast(
        gym.Env[ObsArmsGrCam, TQuartDictType],
        env_cam,
    )


def produce_env_sim_tquart_gripper_camera_cg_rel(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, LimitedCartOrJointContType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_tquart_gripper_camera_cg(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_cam = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedTQuartRelDictType],
        env_cam,
    )


def produce_env_sim_trpy_gripper_camera(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, TRPYDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_sim, info = produce_env_sim_trpy(mjcf_path, urdf_path, cfg, robot_id)
    simulation = info["sim"]
    gripper = sim.FrankaHand(info["sim"], "0", gripper_cfg)
    env_sim = GripperWrapper(env_sim, gripper)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam: gym.Env = CameraSetWrapper(env_sim, camera_set)
    env_cam = GripperWrapper(env_cam, gripper)
    return typing.cast(
        gym.Env[ObsArmsGrCam, TRPYDictType],
        env_cam,
    )


def produce_env_sim_trpy_gripper_camera_rel(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, TRPYDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_trpy_gripper_camera(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_cam = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType],
        env_cam,
    )


def produce_env_sim_trpy_gripper_camera_cg(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, TRPYDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_sim = produce_env_sim_trpy_gripper_camera(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_cam = CollisionGuard.env_from_xml_paths(
        env_sim,
        mjcf_path,
        urdf_path,
        gripper=True,
        check_home_collision=False,
    )
    return typing.cast(
        gym.Env[ObsArmsGrCam, TRPYDictType],
        env_cam,
    )


def produce_env_sim_trpy_gripper_camera_cg_rel(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, LimitedTRPYRelDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_trpy_gripper_camera_cg(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_cam = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedTRPYRelDictType],
        env_cam,
    )


def produce_env_sim_joints_gripper_camera(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, JointsDictType]:
    """Environment with tquart, grippter, camera"""
    env_sim, info = produce_env_sim_joints(mjcf_path, urdf_path, cfg, robot_id)
    simulation = info["sim"]
    gripper = sim.FrankaHand(info["sim"], "0", gripper_cfg)
    env_sim = GripperWrapper(env_sim, gripper)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam: gym.Env = CameraSetWrapper(env_sim, camera_set)
    env_cam = GripperWrapper(env_cam, gripper)
    return typing.cast(
        gym.Env[ObsArmsGrCam, JointsDictType],
        env_cam,
    )


def produce_env_sim_joints_gripper_camera_rel(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, JointsDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_joints_gripper_camera(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_cam = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedJointsRelDictType],
        env_cam,
    )


def produce_env_sim_joints_gripper_camera_cg(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, JointsDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_sim = produce_env_sim_joints_gripper_camera(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_cam = CollisionGuard.env_from_xml_paths(
        env_sim,
        mjcf_path,
        urdf_path,
        gripper=True,
        check_home_collision=False,
    )
    return typing.cast(
        gym.Env[ObsArmsGrCam, JointsDictType],
        env_cam,
    )


def produce_env_sim_joints_gripper_camera_cg_rel(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, gripper_cfg: sim.FHConfig,
    cam_cfg: rcsss.camera.sim.SimCameraSetConfig, robot_id="0"
) -> gym.Env[ArmObsType, LimitedJointsRelDictType]:
    """Environment with tquart, gripper, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_joints_gripper_camera_cg(mjcf_path, urdf_path, cfg, gripper_cfg, cam_cfg, robot_id)
    env_cam = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedJointsRelDictType],
        env_cam,
    )


def produce_env_hw(ip: str, urdf_path: str) -> tuple[gym.Env[ArmObsType, CartOrJointContType]]:
    robot = hw.FR3(ip, urdf_path)
    env = FR3Env(robot, ControlMode.JOINTS)
    env_hw = FR3HW(env)
    return typing.cast(gym.Env[ArmObsType, CartOrJointContType], env_hw)


def produce_env_hw_joints():
    pass


def produce_env_sim_joints_gripper_camera_cg_rel():
    pass

# permutations base(sim|hw) x Control(joints|trpy|tquart) x Util(Gripper,Camera,CollisionGuard,RelativeAction|Gripper,Camera,CollisionGuard|Gripper,Camera,RelativeAction)
# 2 + 2x3 + 2x3x3 = 26 permutations
