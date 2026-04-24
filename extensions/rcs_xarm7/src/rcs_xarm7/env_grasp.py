import logging
import math
from time import sleep

from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldXArm7
from rcs.hand.tilburg_hand import THConfig
from rcs_xarm7.configs import DefaultXArm7HardwareEnv

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.1.245"
# ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_INSTANCE = RobotPlatform.HARDWARE


def sim_env():
    scene = EmptyWorldXArm7()
    cfg = scene.config()
    cfg.control_mode = ControlMode.JOINTS
    env_rel = scene.create_env(cfg)
    env_rel.get_wrapper_attr("sim").open_gui()
    return env_rel


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        hand_cfg = THConfig(
            calibration_file="/home/ken/tilburg_hand/calibration.json", grasp_percentage=1, hand_orientation="right"
        )
        env_creator = DefaultXArm7HardwareEnv()
        env_creator.ip = ROBOT_IP
        cfg = env_creator.config()
        cfg.control_mode = ControlMode.JOINTS
        cfg.hand_cfg = hand_cfg
        cfg.relative_to = RelativeTo.LAST_STEP
        cfg.max_relative_movement = None
        env_rel = env_creator.create_env(cfg)
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
