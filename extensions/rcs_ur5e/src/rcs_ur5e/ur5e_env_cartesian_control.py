import logging
from time import sleep

import numpy as np
from rcs._core import common
from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)
# from rcs_fr3.utils import default_fr3_hw_gripper_cfg
from rcs_ur5e.creators import RCSUR5eEnvCreator
from rcs_ur5e.hw import RobotiQGripper, UR5eConfig

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.1.15"
ROBOT_INSTANCE = RobotPlatform.HARDWARE


def main():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        robot_cfg = UR5eConfig()
        robot_cfg.async_control = False
        env_rel = RCSUR5eEnvCreator()(
            robot_cfg=robot_cfg,
            control_mode=ControlMode.CARTESIAN_TRPY,
            ip=ROBOT_IP,
            camera_set=None,
            max_relative_movement=0.2,
            relative_to=RelativeTo.LAST_STEP,
        )
    else:
        robot_cfg = sim.SimRobotConfig()
        robot_cfg.actuators = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        robot_cfg.joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        robot_cfg.base = "base"
        robot_cfg.robot_type = rcs.common.RobotType.UR5e
        robot_cfg.attachment_site = "attachment_site"
        robot_cfg.arm_collision_geoms = []
        env_rel = SimEnvCreator()(
            control_mode=ControlMode.CARTESIAN_TQuat,
            collision_guard=False,
            robot_cfg=robot_cfg,
            gripper_cfg=None,
            # cameras=default_mujoco_cameraset_cfg(),
            max_relative_movement=0.5,
            relative_to=RelativeTo.LAST_STEP,
            # mjcf=rcs.scenes["ur5e_empty_world"]["mjb"],
            mjcf="/home/johannes/repos/learning/robot-control-stack/assets/scenes/ur5e_empty_world/scene.xml",
            # urdf_path=rcs.scenes["ur5e_empty_world"]["urdf"],
            urdf_path="/home/johannes/repos/learning/robot-control-stack/assets/ur5e/urdf/ur5e.urdf",
        )
        env_rel.get_wrapper_attr("sim").open_gui()

    obs, info = env_rel.reset()
    # print(obs)
    # print(env_rel.unwrapped.robot.get_cartesian_position())  # type: ignore
    # print(env_rel.unwrapped.robot.get_joint_position())  # type: ignore


    act = {"xyzrpy": [0.0, 0, 0.0, 0, 0, np.deg2rad(45)], "gripper": 0}
    obs, reward, terminated, truncated, info = env_rel.step(act)
    #sleep(10)
    exit()
    

    for _ in range(100):
        # act = {"joints": common.robots_meta_config(common.RobotType.UR5e).q_home, "gripper": 0}
        # env_rel.step(act)
        # env_rel.unwrapped.robot.move_home()
        # simulation = env_rel.get_wrapper_attr("sim")
        # simulation.step_until_convergence()
        # sleep(1)
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.01, 0,0, 0.0087265, 0, 0, 0.9999619], "gripper": 0}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            #print(obs)
            # print(env_rel.unwrapped.robot.get_cartesian_position())  # type: ignore
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
            sleep(0.1)
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [-0.01, 0, 0, -0.0087265, 0, 0, 0.9999619], "gripper": 1}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
            sleep(0.1)


if __name__ == "__main__":
    main()
