import logging

import numpy as np
from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import default_sim_robot_cfg, default_sim_tilburg_hand_cfg

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_INSTANCE = RobotPlatform.SIMULATION


def main():
    mjcf = "/home/sbien/Documents/Development/RCS/models/scenes/fr3_tilburg_empty_world/scene.xml"
    if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
        env_rel = SimEnvCreator()(
            control_mode=ControlMode.JOINTS,
            collision_guard=False,
            robot_cfg=default_sim_robot_cfg(),
            hand_cfg=default_sim_tilburg_hand_cfg(),
            # cameras=default_mujoco_cameraset_cfg(),
            mjcf=mjcf,
            max_relative_movement=np.deg2rad(5),
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel.get_wrapper_attr("sim").open_gui()

        for _ in range(10):
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
