import logging

from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():

    env_rel = SimEnvCreator()(
        control_mode=ControlMode.CARTESIAN_TQuat,
        robot_cfg=default_sim_robot_cfg(scene="fr3_empty_world"),
        collision_guard=False,
        gripper_cfg=default_sim_gripper_cfg(),
        cameras=default_mujoco_cameraset_cfg(),
        max_relative_movement=0.5,
        relative_to=RelativeTo.LAST_STEP,
    )
    env_rel.get_wrapper_attr("sim").open_gui()

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
