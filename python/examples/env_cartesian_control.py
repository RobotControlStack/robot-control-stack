import logging

from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import FR3SimEnvCreator, RCSFR3EnvCreator
from rcs.envs.utils import (
    default_fr3_hw_gripper_cfg,
    default_fr3_hw_robot_cfg,
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
)
from rcs_fr3.desk import FCI, ContextManager, Desk, load_creds_fr3_desk

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.101.1"
ROBOT_INSTANCE = RobotPlatform.SIMULATION


"""
Create a .env file in the same directory as this file with the following content:
FR3_USERNAME=<username on franka desk>
FR3_PASSWORD=<password on franka desk>

When you use a real FR3 you first need to unlock its joints using the following cli script:

python -m rcs.fr3 unlock <ip>

or put it into guiding mode using:

python -m rcs.fr3 guiding-mode <ip>

When you are done you lock it again using:

python -m rcs.fr3 lock <ip>

or even shut it down using:

python -m rcs.fr3 shutdown <ip>
"""


def main():
    context_manger: ContextManager
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        user, pw = load_creds_fr3_desk()
        context_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False)
    else:
        context_manger = ContextManager()

    with context_manger:
        if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
            env_rel = RCSFR3EnvCreator()(
                ip=ROBOT_IP,
                control_mode=ControlMode.CARTESIAN_TQuat,
                robot_cfg=default_fr3_hw_robot_cfg(),
                collision_guard="lab",
                gripper_cfg=default_fr3_hw_gripper_cfg(),
                max_relative_movement=0.5,
                relative_to=RelativeTo.LAST_STEP,
            )
        else:
            env_rel = FR3SimEnvCreator()(
                control_mode=ControlMode.CARTESIAN_TQuat,
                robot_cfg=default_fr3_sim_robot_cfg(),
                collision_guard=False,
                gripper_cfg=default_fr3_sim_gripper_cfg(),
                cameras=default_mujoco_cameraset_cfg(),
                max_relative_movement=0.5,
                relative_to=RelativeTo.LAST_STEP,
            )
            env_rel.get_wrapper_attr("sim").open_gui()

        env_rel.reset()
        print(env_rel.unwrapped.robot.get_cartesian_position())  # type: ignore

        for _ in range(10):
            for _ in range(10):
                # move 1cm in x direction (forward) and close gripper
                act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": 0}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return
            for _ in range(10):
                # move 1cm in negative x direction (backward) and open gripper
                act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": 1}
                obs, reward, terminated, truncated, info = env_rel.step(act)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return


if __name__ == "__main__":
    main()
