import logging

from dotenv import dotenv_values
from rcsss.desk import FCI, Desk, DummyResourceManager
from rcsss.envs.base import ControlMode, RobotInstance

from common import hw_env_rel, sim_env_rel

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
        env_rel = hw_env_rel(ROBOT_IP, ControlMode.CARTESIAN_TQuart)
        creds = dotenv_values()
        resource_manger = FCI(
            Desk(ROBOT_IP, creds["FR3_USERNAME"], creds["FR3_USERNAME"]), unlock=False, lock_when_done=False
        )
    else:
        env_rel = sim_env_rel(ControlMode.CARTESIAN_TQuart)
        resource_manger = DummyResourceManager()

    print(env_rel.unwrapped.robot.get_cartesian_position())

    with resource_manger:
        for _ in range(10):
            for _ in range(10):
                act = {"tquart": [0.01, 0, 0, 0, 0, 0, 1], "gripper": 0}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return
            for _ in range(10):
                act = {"tquart": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": 1}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return


if __name__ == "__main__":
    main()
