import logging
from time import sleep

import numpy as np
from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs_ur5e.creators import RCSUR5eEnvCreator
from rcs_ur5e.hw import UR5eConfig

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.1.15"
ROBOT_INSTANCE = RobotPlatform.SIMULATION  # Change to RobotPlatform.HARDWARE for real robot


def main():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        robot_cfg = UR5eConfig()
        robot_cfg.async_control = False
        env_rel = RCSUR5eEnvCreator()(
            robot_cfg=robot_cfg,
            control_mode=ControlMode.CARTESIAN_TQuat,
            ip=ROBOT_IP,
            camera_set=None,
            max_relative_movement=0.2,
            relative_to=RelativeTo.LAST_STEP,
        )
    else:
        robot_sim_cfg = sim.SimRobotConfig()
        robot_sim_cfg.actuators = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        robot_sim_cfg.joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        robot_sim_cfg.robot_type = rcs.common.RobotType.UR5e
        robot_sim_cfg.attachment_site = "attachment_site"
        robot_sim_cfg.arm_collision_geoms = []
        scene = rcs.scenes["ur5e_empty_world"]
        robot_sim_cfg.mjcf_scene_path = scene.mjb or scene.mjcf_scene
        robot_sim_cfg.kinematic_model_path = rcs.scenes["ur5e_empty_world"].mjcf_robot
        robot_sim_cfg.base = "base"
        robot_sim_cfg.tcp_offset = rcs.common.Pose()

        gripper_config = sim.SimGripperConfig()
        gripper_config.actuator = "fingers_actuator"
        gripper_config.joints = ["right_driver_joint"]
        gripper_config.collision_geoms = []
        gripper_config.collision_geoms_fingers = []
        gripper_config.max_actuator_width = 0
        gripper_config.min_actuator_width = 1
        gripper_config.max_joint_width = 0.0
        gripper_config.min_joint_width = 0.8

        env_rel = SimEnvCreator()(
            control_mode=ControlMode.CARTESIAN_TQuat,
            collision_guard=False,
            robot_cfg=robot_sim_cfg,
            gripper_cfg=gripper_config,
            max_relative_movement=(0.1, np.deg2rad(5)),
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel.get_wrapper_attr("sim").open_gui()

    obs, info = env_rel.reset()

    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1.0], "gripper": [0]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            sleep(0.6)
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1.0], "gripper": [1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            sleep(0.6)


if __name__ == "__main__":
    main()
