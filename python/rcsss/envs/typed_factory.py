import typing
import rcsss
from rcsss.envs.sim import CollisionGuard
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperWrapper,
    RelativeActionSpace,
    ObsArmsGrCam,
    LimitedCartOrJointContType
)
from rcsss import sim
from rcsss._core.sim import CameraType, SimCameraSet, SimCameraSetConfig
from rcsss.camera.sim import SimCameraConfig
from rcsss.envs.base import ArmObsType, CameraSetWrapper, CartOrJointContType, ControlMode, FR3Env, JointsDictType, TQuartDictType, TRPYDictType
from rcsss.envs.sim import FR3Sim
import gymnasium as gym


def produce_env_sim(
    mjcf_path: str, urdf_path: str, control_mode: ControlMode, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, CartOrJointContType]:
    simulation = sim.Sim(mjcf_path)
    robot = sim.FR3(simulation, robot_id, urdf_path)
    robot.set_parameters(cfg)
    env = FR3Env(robot, control_mode)
    env_sim = FR3Sim(env, simulation)
    return typing.cast(gym.Env[ArmObsType, CartOrJointContType], FR3Sim(env_sim, simulation))


def produce_env_sim_joints(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, JointsDictType]:
    return typing.cast(
        gym.Env[ArmObsType, JointsDictType],
        produce_env_sim(mjcf_path, urdf_path, control_mode=ControlMode.JOINTS, cfg=cfg, robot_id=robot_id),
    )


def produce_env_sim_trpy(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, TRPYDictType]:
    return typing.cast(
        gym.Env[ArmObsType, TRPYDictType],
        produce_env_sim(mjcf_path, urdf_path, control_mode=ControlMode.CARTESIAN_TRPY, cfg=cfg, robot_id=robot_id),
    )


def produce_env_sim_tquart(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    return typing.cast(
        gym.Env[ArmObsType, TQuartDictType],
        produce_env_sim(mjcf_path, urdf_path, control_mode=ControlMode.CARTESIAN_TQuart, cfg=cfg, robot_id=robot_id),
    )


def produce_env_sim_tquart_gripper_camera(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    simulation = sim.Sim(mjcf_path)
    env_sim = produce_env_sim_tquart(mjcf_path, urdf_path, cfg, robot_id)
    gripper_cfg = sim.FHConfig()
    gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
    env_sim = GripperWrapper(env_sim, gripper)
    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
    }
    cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=50)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam: gym.Env = CameraSetWrapper(env_sim, camera_set)

    gripper_cfg = sim.FHConfig()
    gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
    env_cam = GripperWrapper(env_cam, gripper)
    return typing.cast(
        gym.Env[ObsArmsGrCam, TQuartDictType],
        env_cam,
    )


def produce_env_sim_tquart_gripper_camera_rel(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_tquart_gripper_camera(mjcf_path, urdf_path, cfg, robot_id)
    env_cam = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedCartOrJointContType],
        env_cam,
    )


def produce_env_sim_tquart_gripper_camera_cg(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_sim = produce_env_sim_tquart_gripper_camera(mjcf_path, urdf_path, cfg, robot_id)
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
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, LimitedCartOrJointContType]:
    """Environment with tquart, grippter, camera, collision guard and relative limits"""
    env_cam = produce_env_sim_tquart_gripper_camera_cg(mjcf_path, urdf_path, cfg, robot_id)
    env_cam = RelativeActionSpace(env_cam)
    return typing.cast(
        gym.Env[ObsArmsGrCam, LimitedCartOrJointContType],
        env_cam,
    )


def produce_env_hw():
    pass


def produce_env_hw_joints():
    pass


def produce_env_sim_joints_gripper_camera_cg_rel():
    pass

# permutations base(sim|hw) x Control(joints|trpy|tquart) x Util(Gripper,Camera,CollisionGuard,RelativeAction|Gripper,Camera,CollisionGuard|Gripper,Camera,RelativeAction)
# 2 + 2x3 + 2x3x3 = 26 permutations
