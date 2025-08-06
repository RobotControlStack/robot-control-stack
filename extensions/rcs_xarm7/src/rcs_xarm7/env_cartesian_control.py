import logging
from time import sleep

from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs_fr3.desk import ContextManager
from rcs_xarm7.creators import RCSXArm7EnvCreator, XArm7SimEnvCreator

import rcs
from rcs import sim
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.1.245"
# ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_INSTANCE = RobotPlatform.HARDWARE


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        env_rel = RCSXArm7EnvCreator()(
            control_mode=ControlMode.CARTESIAN_TQuat,
            ip=ROBOT_IP,
            relative_to=RelativeTo.LAST_STEP,
            max_relative_movement=np.deg2rad(3)
        )
        cm = env_rel.unwrapped.robot
    else:
        cm = ContextManager()
        cfg = sim.SimRobotConfig()
        cfg.robot_type = rcs.common.RobotType.XArm7
        cfg.actuators = ["1", "2", "3", "4", "5"]
        cfg.joints = ["1", "2", "3", "4", "5"]
        cfg.arm_collision_geoms = []
        cfg.attachment_site = "gripper"

        grp_cfg = sim.SimGripperConfig()
        grp_cfg.actuator = "6"
        grp_cfg.joint = "6"
        grp_cfg.collision_geoms = []
        grp_cfg.collision_geoms_fingers = []

        env_rel = XArm7SimEnvCreator()(
            control_mode=ControlMode.JOINTS,
            urdf_path="/home/tobi/coding/lerobot/so101_new_calib.urdf",
            robot_cfg=cfg,
            collision_guard=False,
            mjcf="/home/tobi/coding/lerobot/SO-ARM100/Simulation/XArm7/scene.xml",
            gripper_cfg=grp_cfg,
            # camera_set_cfg=default_mujoco_cameraset_cfg(),
            max_relative_movement=None,
            # max_relative_movement=10.0,
            # max_relative_movement=0.5,
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel.get_wrapper_attr("sim").open_gui()
    env_rel.reset()
    act = {"tquat": [0.1, 0, 0, 0, 0, 0, 1], "gripper": 0}
    obs, reward, terminated, truncated, info = env_rel.step(act)

    with cm:
        for _ in range(10):
            for _ in range(10):
                # move 1cm in x direction (forward) and close gripper
                act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": 0}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return
                print(obs)
                #sleep(2.0)
            for _ in range(10):
                # move 1cm in negative x direction (backward) and open gripper
                act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": 1}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return


if __name__ == "__main__":
    main()
