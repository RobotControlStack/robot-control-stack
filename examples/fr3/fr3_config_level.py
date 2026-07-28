import logging

import numpy as np
from rcs._core.common import RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.base import ControlMode, RelativeTo, SimpleFrameRate
from rcs.envs.configs import EmptyWorldFR3
from rcs.envs.tasks import PickTaskConfig


logger = logging.getLogger(__name__)


ROBOT_IP = "192.168.102.1"
ROBOT_INSTANCE = RobotPlatform.SIMULATION
FPS = 30



def get_env():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_fr3.configs import DefaultFR3HardwareEnv

        env_creator = DefaultFR3HardwareEnv()
        env_creator.ip = ROBOT_IP
        hw_cfg = env_creator.config()
        hw_cfg.control_mode = ControlMode.JOINTS
        hw_cfg.wrapper_cfg.binary_gripper = True
        hw_cfg.max_relative_movement = np.deg2rad(5)
        hw_cfg.relative_to = RelativeTo.LAST_STEP
        hw_cfg.robot_cfgs["right"].ignore_realtime = True
        hw_cfg.robot_cfgs["right"].speed_factor = 0.3
        hw_cfg.robot_cfgs["right"].async_control = True
        env_rel = env_creator.create_env(hw_cfg)
    else:
        # FR3

        scene = EmptyWorldFR3()
        sim_cfg_data = scene.config()
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True, realtime=True, frequency=FPS, max_convergence_steps=500
        )
        sim_cfg_data.relative_to = RelativeTo.LAST_STEP
        sim_cfg_data.task_cfg = PickTaskConfig(robot_name="right")

        env_rel = scene.create_env(sim_cfg_data)

        sim = env_rel.get_wrapper_attr("sim")

    return env_rel


def main():
    frame_rate = SimpleFrameRate(FPS)
    env_rel = get_env()
    env_rel.reset()
    for _ in range(100):
        obs, info = env_rel.reset()
        for _ in range(10):
            # sample random relative action and execute it
            act = env_rel.action_space.sample()
            obs, reward, terminated, truncated, info = env_rel.step(act)
            frame_rate()


if __name__ == "__main__":
    main()
