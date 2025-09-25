import logging
import math
from time import sleep

from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import default_sim_tilburg_hand_cfg
from rcs.hand.tilburg_hand import THConfig
from rcs_xarm7.creators import RCSXArm7EnvCreator

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.1.245"
# ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_INSTANCE = RobotPlatform.HARDWARE


def sim_env():
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
    env_rel = SimEnvCreator()(
        robot_cfg=robot_cfg,
        control_mode=ControlMode.JOINTS,
        gripper_cfg=None,
        hand_cfg=default_sim_tilburg_hand_cfg(),
        # cameras=default_mujoco_cameraset_cfg(),
        # max_relative_movement=0.5,
        relative_to=RelativeTo.LAST_STEP,
    )
    env_rel.get_wrapper_attr("sim").open_gui()
    return env_rel


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        hand_cfg = THConfig(
            calibration_file="/home/ken/tilburg_hand/calibration.json", grasp_percentage=1, hand_orientation="right"
        )
        env_rel = RCSXArm7EnvCreator()(
            control_mode=ControlMode.JOINTS,
            ip=ROBOT_IP,
            hand_cfg=hand_cfg,
            relative_to=RelativeTo.LAST_STEP,
            max_relative_movement=None,
        )
    else:
        env_rel = sim_env()

    twin_env = sim_env()

    env_rel.reset()

    actions = [
        # open hand
        ([0, math.radians(-45), 0, math.radians(15), 0, math.radians(-25), 0], 1, 2.0),
        # approach
        ([0, math.radians(45), 0, math.radians(40), 0, math.radians(-95), 0], 1, 2.0),
        # close hand
        ([0, math.radians(45), 0, math.radians(40), 0, math.radians(-95), 0], 0, 2.0),
        # lift
        ([0, math.radians(15), 0, math.radians(30), 0, math.radians(-75), 0], 0, 4.0),
        # put back
        ([0, math.radians(45), 0, math.radians(40), 0, math.radians(-95), 0], 0, 2.0),
        # open hand
        ([0, math.radians(45), 0, math.radians(40), 0, math.radians(-95), 0], 1, 2.0),
        # back to home
        ([0, math.radians(-45), 0, math.radians(15), 0, math.radians(-25), 0], 1, 0.0),
    ]

    with env_rel:
        for joints, hand, delay in actions:
            act = {"joints": joints, "hand": hand}
            twin_env.step(act)
            obs, reward, terminated, truncated, info = env_rel.step(act)
            # twin_robot.set_joint_position(joints)
            # twin_sim.step(50)
            sleep(1)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                break
            if delay > 0:
                sleep(delay)


if __name__ == "__main__":
    main()
