import typing
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

def produce_env_sim_tquart_gripper_camera_cg_rel(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    pass

def produce_env_sim_tquart_gripper_camera_cg(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    pass

def produce_env_sim_tquart_gripper_camera_rel(
    mjcf_path: str, urdf_path: str, cfg: sim.FR3Config, robot_id="0"
) -> gym.Env[ArmObsType, TQuartDictType]:
    pass

# ...


def produce_env_hw():
    pass

def produce_env_hw_joints():
    pass

# ...

def produce_env_sim_joints_gripper_camera_cg_rel():
    pass

# permutations base(sim|hw) x Control(joints|trpy|tquart) x Util(Gripper,Camera,CollisionGuard,RelativeAction|Gripper,Camera,CollisionGuard|Gripper,Camera,RelativeAction)
# 2 + 2x3 + 2x3x3 = 26 permutations