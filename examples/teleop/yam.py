import logging

import numpy as np
from rcs._core.common import BaseCameraConfig, RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.configs import EmptyWorldYam
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.operator.interface import TeleopLoop
from rcs.operator.quest import QuestConfig, QuestOperator
from simpub.sim.mj_publisher import MujocoPublisher

logger = logging.getLogger(__name__)

"""
Teleoperation of two YAM arms with the Meta Quest 3. See README.md for the setup of the quest and
the IRIS app; each arm follows its controller while the trigger is held, and the hand trigger
drives the gripper.

To teleoperate real hardware, install the rcs_yam extension (`pip install -ve extensions/rcs_yam`),
bring up the CAN interfaces (`sudo ip link set can0 up type can bitrate 1000000`, same for can1)
and set ROBOT_INSTANCE to RobotPlatform.HARDWARE.

RealSense cameras are recorded on hardware when CAMERA_DICT is set, which needs the rcs_realsense
extension (`pip install -ve extensions/rcs_realsense`).
"""

ROBOT_INSTANCE = RobotPlatform.HARDWARE
# The teleop loop matches robots to controllers by name: each arm is named after the controller
# that drives it.
CAN_CHANNELS = {"left": "can0", "right": "can1"}

MQ3_ADDR = "10.42.0.1"
RECORD_FPS = 30

# Serial numbers of the RealSense cameras, use `rs-enumerate-devices -s` to list them.
# Set CAMERA_DICT to None to disable cameras.
CAMERA_DICT = {
    "right_wrist": "230422272017",
    "left_wrist": "230422271040",
}
# CAMERA_DICT = None
INCLUDE_DEPTH = False

DATASET_PATH = "yam_teleop"
INSTRUCTION = "pick up cube"

config = QuestConfig(
    mq3_addr=MQ3_ADDR,
    simulation=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
    switched_left_right=True,
    display_cameras=False,
)


def get_env():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_yam.configs import DefaultYamDualMultiHardwareEnv
        from rcs_yam.creators import HardwareCameraCreatorConfig

        env_creator = DefaultYamDualMultiHardwareEnv()
        env_creator.left_channel = CAN_CHANNELS["left"]
        env_creator.right_channel = CAN_CHANNELS["right"]
        # The dual config already enables async_control, so the setters do not wait for the arms.
        hw_cfg = env_creator.config()
        camera_cfgs: dict[str, HardwareCameraCreatorConfig] = {}
        if CAMERA_DICT is not None:
            camera_cfgs["realsense"] = HardwareCameraCreatorConfig(
                camera_type_id="realsense",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=1280,
                        resolution_height=720,
                        frame_rate=30,
                    )
                    for name, identifier in CAMERA_DICT.items()
                },
            )
        hw_cfg.camera_cfgs = camera_cfgs or None
        hw_cfg.control_mode = config.operator_class.control_mode[0]
        hw_cfg.relative_to = config.operator_class.control_mode[1]
        hw_cfg.max_relative_movement = (0.5, np.deg2rad(90))
        hw_cfg.wrapper_cfg.binary_gripper = False
        hw_cfg.wrapper_cfg.include_depth = INCLUDE_DEPTH
        env_rel = env_creator.create_env(hw_cfg)
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
