import logging

import numpy as np
from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import get_tcp_offset
from rcs_xarm7.creators import RCSXArm7EnvCreator, XArm7SimEnvCreator
from rcs.hand.tilburg_hand import THConfig
import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.1.245"
# ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_INSTANCE = RobotPlatform.HARDWARE


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        hand_cfg = THConfig(
            calibration_file="/home/ken/tilburg_hand/calibration.json",
            grasp_percentage=1,
            hand_orientation="right"
        )
        env_rel = RCSXArm7EnvCreator()(
            control_mode=ControlMode.CARTESIAN_TQuat,
            ip=ROBOT_IP,
            hand_cfg=hand_cfg,
            relative_to=RelativeTo.LAST_STEP,
            max_relative_movement=np.deg2rad(3),
        )
    else:
        robot_cfg = sim.SimRobotConfig()
        robot_cfg.actuators = [
            "act1",
            "act2",
            "act3",
            "act4",
            "act5",
            "act6",
            "act7",
        ]
        robot_cfg.joints = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        robot_cfg.base = "base"
        robot_cfg.robot_type = rcs.common.RobotType.XArm7
        robot_cfg.attachment_site = "attachment_site"
        robot_cfg.arm_collision_geoms = []
        robot_cfg.tcp_offset = get_tcp_offset(rcs.scenes["xarm7_empty_world"]["mjcf_robot"])
        env_rel = SimEnvCreator()(
            control_mode=ControlMode.CARTESIAN_TQuat,
            collision_guard=False,
            robot_cfg=robot_cfg,
            gripper_cfg=None,
            # cameras=default_mujoco_cameraset_cfg(),
            max_relative_movement=0.5,
            relative_to=RelativeTo.LAST_STEP,
            mjcf=rcs.scenes["xarm7_empty_world"]["mjb"],
            robot_kinematics_path=rcs.scenes["xarm7_empty_world"]["mjcf_robot"],
        )
        env_rel.get_wrapper_attr("sim").open_gui()

    env_rel.reset()
    # act = {"tquat": [0.1, 0, 0, 0, 0, 0, 1], "gripper": 0}
    # obs, reward, terminated, truncated, info = env_rel.step(act)

    with env_rel:
        for _ in range(10):
            for _ in range(10):
                # move 1cm in x direction (forward) and close gripper
                act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "hand": 1}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return
            for _ in range(10):
                # move 1cm in negative x direction (backward) and open gripper
                act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "hand": 0}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return


if __name__ == "__main__":
    main()
