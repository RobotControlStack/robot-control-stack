import logging

import numpy as np
from rcs.envs.base import ControlMode, RelativeTo

from rcs_panda.creators import RCSFR3EnvCreator
from rcs_panda.utils import default_fr3_hw_gripper_cfg, default_fr3_hw_robot_cfg

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():
    # TODO!
    # env_rel = SimEnvCreator()(
    #     control_mode=ControlMode.JOINTS,
    #     collision_guard=False,
    #     robot_cfg=default_sim_robot_cfg("fr3_empty_world"),
    #     gripper_cfg=default_sim_gripper_cfg(),
    #     cameras=default_mujoco_cameraset_cfg(),
    #     max_relative_movement=np.deg2rad(5),
    #     relative_to=RelativeTo.LAST_STEP,
    # )
    # env_rel.get_wrapper_attr("sim").open_gui()
    env = RCSFR3EnvCreator()
    env_rel = env(
        ip="192.168.4.100",
        control_mode=ControlMode.JOINTS,
        robot_cfg=default_fr3_hw_robot_cfg(),
        collision_guard=None,
        gripper_cfg=default_fr3_hw_gripper_cfg(),
        camera_set=None,
        max_relative_movement=(np.deg2rad(5)),
        relative_to=RelativeTo.LAST_STEP,
    )
    input("moving")


    for _ in range(100):
        obs, info = env_rel.reset()
        for _ in range(10):
            # sample random relative action and execute it
            act = env_rel.action_space.sample()
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return


if __name__ == "__main__":
    main()
