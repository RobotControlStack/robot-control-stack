import logging

import numpy as np
from rcs._core.common import RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.base import CoverWrapper, MultiRobotWrapper
from rcs.envs.configs import EmptyWorldYam
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.operator.interface import TeleopLoop
from rcs.operator.quest import QuestConfig, QuestOperator
from simpub.sim.mj_publisher import MujocoPublisher

logger = logging.getLogger(__name__)

"""
Teleoperation of a single YAM arm with the Meta Quest 3, without cameras. See README.md for the setup
of the quest and the IRIS app; the arm follows the right controller while its trigger is held, and the
hand trigger drives the gripper.

To teleoperate real hardware, install the rcs_yam extension (`pip install -ve extensions/rcs_yam`),
bring up the CAN interface (`sudo ip link set can0 up type can bitrate 1000000`) and set
ROBOT_INSTANCE to RobotPlatform.HARDWARE.
"""

ROBOT_INSTANCE = RobotPlatform.SIMULATION
CAN_CHANNEL = "can0"
# The teleop loop matches robots to controllers by name, so the arm is named after the right one.
ROBOT_NAME = "right"

MQ3_ADDR = "10.42.0.1"
RECORD_FPS = 30

DATASET_PATH = "yam_teleop"
INSTRUCTION = "pick up cube"

config = QuestConfig(
    mq3_addr=MQ3_ADDR,
    simulation=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
    switched_left_right=False,
    display_cameras=False,
)


def get_env():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_yam.configs import DefaultYamHardwareEnv

        env_creator = DefaultYamHardwareEnv()
        env_creator.channel = CAN_CHANNEL
        hw_cfg = env_creator.config()
        hw_cfg.control_mode = config.operator_class.control_mode[0]
        hw_cfg.relative_to = config.operator_class.control_mode[1]
        hw_cfg.max_relative_movement = (0.5, np.deg2rad(90))
        hw_cfg.wrapper_cfg.binary_gripper = False
        # Targets arrive at RECORD_FPS, so the setters must not wait for the arm to arrive.
        hw_cfg.robot_cfg.async_control = True
        # Even a single arm goes through MultiRobotWrapper, since the teleop loop steps per robot name.
        env_rel = CoverWrapper(MultiRobotWrapper({ROBOT_NAME: env_creator.create_env(hw_cfg)}))
        operator = QuestOperator(config)
    else:
        scene = EmptyWorldYam()
        sim_cfg_data = scene.config()
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True, realtime=True, frequency=RECORD_FPS, max_convergence_steps=500
        )
        sim_cfg_data.control_mode = config.operator_class.control_mode[0]
        sim_cfg_data.relative_to = config.operator_class.control_mode[1]
        sim_cfg_data.max_relative_movement = (0.5, np.deg2rad(90))
        sim_cfg_data.wrapper_cfg.binary_gripper = False
        env_rel = scene.create_env(sim_cfg_data)

        sim = env_rel.get_wrapper_attr("sim")
        MujocoPublisher(sim.model, sim.data, MQ3_ADDR, visible_geoms_groups=list(range(1, 3)))
        operator = QuestOperator(config, sim)

    env_rel = StorageWrapper(
        env_rel, DATASET_PATH, INSTRUCTION, batch_size=32, max_rows_per_group=100, max_rows_per_file=1000
    )
    return env_rel, operator


def main():
    env_rel, operator = get_env()
    env_rel.reset()
    tele = TeleopLoop(env_rel, operator, env_frequency=RECORD_FPS, robot_platform=ROBOT_INSTANCE)
    with env_rel, tele:  # type: ignore
        tele.environment_step_loop()


if __name__ == "__main__":
    main()
