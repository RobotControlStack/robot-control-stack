import logging

from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

"""
This script demonstrates how to control the FR3 robot in Cartesian position control mode
using relative movements. The robot (or its simulation) moves 1cm forward and then 1cm backward
in a loop while opening and closing the gripper.

To control a real FR3 robot, install the rcs_fr3 extension (`pip install extensions/rcs_fr3`),
and set the FR3_IP variable to the robot's IP address. Make sure to unlock the robot's joints and
put it into FCI mode before running this script. For a scripted way of unlocking and guiding mode see the
fr3_direct_control.py example which uses the FCI context manager.
"""

ROBOT_INSTANCE = RobotPlatform.SIMULATION
FR3_IP = "192.168.101.1"


def main():
    if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
        env_rel = SimEnvCreator()(
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfg=default_sim_robot_cfg(scene="fr3_empty_world"),
            gripper_cfg=default_sim_gripper_cfg(),
            cameras=default_mujoco_cameraset_cfg(),
            max_relative_movement=0.5,
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel.get_wrapper_attr("sim").open_gui()
    else:
        from rcs_fr3.creators import RCSFR3EnvCreator
        from rcs_fr3.utils import default_fr3_hw_gripper_cfg, default_fr3_hw_robot_cfg

        env_rel = RCSFR3EnvCreator()(
            ip=FR3_IP,
            control_mode=ControlMode.CARTESIAN_TQuat,
            robot_cfg=default_fr3_hw_robot_cfg(),
            gripper_cfg=default_fr3_hw_gripper_cfg(),
            camera_set=None,
            max_relative_movement=0.5,
            relative_to=RelativeTo.LAST_STEP,
        )
        input("the robot is going to move, press enter whenever you are ready")

    env_rel.reset()

    # access low level robot api to get current cartesian position
    print(env_rel.unwrapped.robot.get_cartesian_position())  # type: ignore

    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": 0}
            obs, reward, terminated, truncated, info = env_rel.step(act)
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": 1}
            obs, reward, terminated, truncated, info = env_rel.step(act)


if __name__ == "__main__":
    main()
