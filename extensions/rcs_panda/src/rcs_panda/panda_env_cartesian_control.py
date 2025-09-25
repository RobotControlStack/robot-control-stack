import logging

from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)
from rcs_panda.creators import RCSFR3EnvCreator
from rcs_panda.utils import default_fr3_hw_gripper_cfg, default_fr3_hw_robot_cfg

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():

    env = RCSFR3EnvCreator()
    env_rel = env(
        ip="192.168.4.100",
        control_mode=ControlMode.CARTESIAN_TQuat,
        robot_cfg=default_fr3_hw_robot_cfg(),
        collision_guard=None,
        gripper_cfg=default_fr3_hw_gripper_cfg(),
        camera_set=None,
        max_relative_movement=0.2,
        relative_to=RelativeTo.LAST_STEP,
    )
    input("moving")

    env_rel.reset()
    print(env_rel.unwrapped.robot.get_cartesian_position())  # type: ignore

    for _ in range(100):
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
