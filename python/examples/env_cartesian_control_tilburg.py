import logging

from rcsss.control.fr3_desk import FCI, Desk, DummyResourceManager
from rcsss.control.utils import load_creds_fr3_desk
from rcsss.envs.base import ControlMode, RelativeTo, RobotInstance
from rcsss.envs.factories import fr3_hw_env, fr3_sim_env
from rcsss.envs.utils import (
    default_fr3_hw_robot_cfg,
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
    default_tilburg_hw_hand_cfg,
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
    resource_manger: FCI | DummyResourceManager
    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        user, pw = load_creds_fr3_desk()
        resource_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False)
    else:
        resource_manger = DummyResourceManager()
    with resource_manger:
        if ROBOT_INSTANCE == RobotInstance.HARDWARE:
            env_rel = fr3_hw_env(
                ip=ROBOT_IP,
                control_mode=ControlMode.CARTESIAN_TQuart,
                robot_cfg=default_fr3_hw_robot_cfg(),
                collision_guard="lab",
                gripper_cfg=default_tilburg_hw_hand_cfg(),
                max_relative_movement=0.5,
                relative_to=RelativeTo.LAST_STEP,
            )
        else:
            env_rel = fr3_sim_env(
                control_mode=ControlMode.CARTESIAN_TQuart,
                robot_cfg=default_fr3_sim_robot_cfg(),
                collision_guard=False,
                gripper_cfg=default_fr3_sim_gripper_cfg(),
                camera_set_cfg=default_mujoco_cameraset_cfg(),
                max_relative_movement=0.5,
                relative_to=RelativeTo.LAST_STEP,
            )
            env_rel.get_wrapper_attr("sim").open_gui()

        env_rel.reset()
        print(env_rel.unwrapped.robot.get_cartesian_position())  # type: ignore
        close_action = 0
        open_action = 1

        for _ in range(10):
            for _ in range(10):
                # move 1cm in x direction (forward) and close gripper
                act = {"tquart": [0.01, 0, 0, 0, 0, 0, 1], "hand": close_action}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return
            from time import sleep

            sleep(5)
            for _ in range(10):
                # move 1cm in negative x direction (backward) and open gripper
                act = {"tquart": [-0.01, 0, 0, 0, 0, 0, 1], "hand": open_action}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return


if __name__ == "__main__":
    main()
