import logging
import mujoco
import rcsss

from dotenv import dotenv_values
from rcsss.desk import FCI, Desk, DummyResourceManager
from rcsss.envs.base import ControlMode, RobotInstance

from env_common import hw_env_rel, sim_env_rel

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.101.1"
ROBOT_INSTANCE = RobotInstance.SIMULATION


"""
Create a .env file in the same directory as this file with the following content:
FR3_USERNAME=<username on franka desk>
FR3_PASSWORD=<password on franka desk>
"""


def main():
    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        creds = dotenv_values()
        resource_manger = FCI(
            Desk(ROBOT_IP, creds["FR3_USERNAME"], creds["FR3_PASSWORD"]), unlock=False, lock_when_done=False
        )
    else:
        resource_manger = DummyResourceManager()
    with resource_manger:

        if ROBOT_INSTANCE == RobotInstance.HARDWARE:
            env_rel = hw_env_rel(ROBOT_IP, ControlMode.JOINTS)
        else:
            env_rel = sim_env_rel(ControlMode.JOINTS)

        for _ in range(10):
            obs, info = env_rel.reset()
            for _ in range(3):
                # sample random relative action and execute it
                act = env_rel.action_space.sample()
                # if the first is open, then it does not open
                act["gripper"] = 1
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return
                logger.info(act["gripper"], obs["gripper"])


if __name__ == "__main__":
    main()
