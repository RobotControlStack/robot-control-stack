from itertools import cycle, islice, repeat
from time import sleep

import numpy as np
from rcsss.envs.base import ControlMode, RelativeTo, RobotInstance
from rcsss.envs.factories import (
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
    fr3_sim_env,
)

# ISO CUBE has size  0.4 x 0.4 x 0.4 and is centered at 0.498, 0.0, 0.226
# the top 4 corners are:
# 0.298 -0.2 0.426
# 0.698 -0.2 0.426
# 0.698 0.2 0.426
# 0.298 0.2 0.426
# the lower 4 corners are:
# 0.298 -0.2 0.026
# 0.698 -0.2 0.026
# 0.698 0.2 0.026
# 0.298 0.2 0.026
path = np.array(
    [
        [0.298, -0.2, 0.426],
        [0.698, -0.2, 0.426],
        [0.698, 0.2, 0.426],
        [0.298, 0.2, 0.426],
        [0.298, -0.2, 0.426],
        [0.298, -0.2, 0.026],
        [0.698, -0.2, 0.026],
        [0.698, 0.2, 0.026],
        [0.298, 0.2, 0.026],
        [0.298, -0.2, 0.026],
    ]
)

if __name__ == "__main__":
    env = fr3_sim_env(
        control_mode=ControlMode.CARTESIAN_TQuart,
        robot_cfg=default_fr3_sim_robot_cfg(),
        gripper_cfg=default_fr3_sim_gripper_cfg(),
        camera_set_cfg=default_mujoco_cameraset_cfg(),
    )
    env.get_wrapper_attr("sim").open_gui()
    obs, info = env.reset()
    quaternion = np.array([1, 0, 0, 0])
    gripper = np.zeros(1)
    actions = (
        {
            "tquart": np.concatenate((translation, quaternion)),
            "gripper": gripper
        } for translation in path
    )
    for action in islice(cycle(actions), 10):
        obs, reward, terminated, truncated, info = env.step(action)
        sleep(0.5)
        if truncated or terminated:
            env.reset()
