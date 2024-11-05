import logging

import mujoco
import numpy as np
import rcsss
from dotenv import dotenv_values
from rcsss.control.fr3_desk import FCI, Desk, DummyResourceManager
from rcsss.control.utils import load_creds_fr3_desk
from rcsss.envs.base import ControlMode, RelativeTo, RobotInstance
from rcsss.envs.factories import (
    default_fr3_hw_gripper_cfg,
    default_fr3_hw_robot_cfg,
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
    fr3_hw_env,
    fr3_sim_env,
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.101.1"
ROBOT_INSTANCE = RobotInstance.SIMULATION


"""
Create a .env file in the same directory as this file with the following content:
FR3_USERNAME=<username on franka desk>
FR3_PASSWORD=<password on franka desk>

When you use a real FR3 you first need to unlock its joints using the following cli script:

python -m rcsss fr3 unlock <ip>

or put it into guiding mode using:

python -m rcsss fr3 guiding-mode <ip>

When you are done you lock it again using:

python -m rcsss fr3 lock <ip>

or even shut it down using:

python -m rcsss fr3 shutdown <ip>
"""


def main():
    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        user, pw = load_creds_fr3_desk()
        resource_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False)
    else:
        resource_manger = DummyResourceManager()
    with resource_manger:

        if ROBOT_INSTANCE == RobotInstance.HARDWARE:
            env_rel = fr3_hw_env(
                ip=ROBOT_IP,
                control_mode=ControlMode.JOINTS,
                robot_cfg=default_fr3_hw_robot_cfg(),
                collision_guard="lab",
                gripper_cfg=default_fr3_hw_gripper_cfg(),
                max_relative_movement=np.deg2rad(5),
                relative_to=RelativeTo.LAST_STEP,
            )
        else:
            env_rel = fr3_sim_env(
                control_mode=ControlMode.JOINTS,
                collision_guard=False,
                robot_cfg=default_fr3_sim_robot_cfg(),
                gripper_cfg=default_fr3_sim_gripper_cfg(),
                camera_set_cfg=default_mujoco_cameraset_cfg(),
                max_relative_movement=np.deg2rad(5),
                relative_to=RelativeTo.LAST_STEP,
            )
            env_rel.get_wrapper_attr("sim").open_gui()

        for _ in range(10):
            obs, info = env_rel.reset()
            for _ in range(3):
                # sample random relative action and execute it
                act = env_rel.action_space.sample()
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return
                logger.info(act["gripper"], obs["gripper"])


if __name__ == "__main__":
    main()
