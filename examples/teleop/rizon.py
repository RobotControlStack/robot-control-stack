import logging

import numpy as np
from rcs._core.common import RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.configs import EmptyWorldRizon4SDuo
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.operator.interface import TeleopLoop
from rcs.operator.quest import QuestConfig, QuestOperator
from simpub.sim.mj_publisher import MujocoPublisher

import rcs

logger = logging.getLogger(__name__)

"""
Teleoperation of two Flexiv Rizon 4s arms with the Meta Quest 3. See README.md for the setup of the
quest and the IRIS app; each arm follows its controller while the trigger is held, and the hand
trigger drives the Grav gripper.

The arms stand 30 cm apart in y and are each tilted by 45 degrees about x away from the other arm
(right arm +45 degrees, left arm -45 degrees, like the FR3 duo).
Actions and observations are expressed in the shared base frame between the two bases, which is the
frame the quest axis has to be aligned to (x front, y left, z up). The matching simulation is
registered as `rcs/rizon4s_duo`.

To teleoperate real hardware, install the rcs_flexiv extension (`pip install -ve extensions/rcs_flexiv`),
put both robots into auto mode with released E-stops and set ROBOT_INSTANCE to RobotPlatform.HARDWARE.
"""

ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_SN = {
    "right": "Rizon4s-063650",
}
GRIPPER_DEVICE_NAME = "Flexiv-GN01"
GRIPPER_TOOL_NAME = None  # tool created for the Grav in Flexiv Elements, None keeps the active tool

ROBOT_TO_SHARED_BASE_FRAME = {
    "left": rcs.DEFAULT_TRANSFORMS["RIZON4S_DUO_LEFT_ROBOT"],
    "right": rcs.DEFAULT_TRANSFORMS["RIZON4S_DUO_RIGHT_ROBOT"],
}

MQ3_ADDR = "10.42.0.1"
RECORD_FPS = 30

DATASET_PATH = "rizon_teleop"
INSTRUCTION = "pick up cube"

config = QuestConfig(
    mq3_addr=MQ3_ADDR,
    simulation=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
    switched_left_right=False,
    display_cameras=False,
    read_frequency=90,
)


def get_env():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_flexiv.configs import DefaultRizon4SDualMultiHardwareEnv
        from rcs_flexiv.hw import FlexivControlMode

        env_creator = DefaultRizon4SDualMultiHardwareEnv()
        env_creator.left_sn = ROBOT_SN.get("left", env_creator.left_sn)
        env_creator.right_sn = ROBOT_SN.get("right", env_creator.right_sn)
        env_creator.gripper_device_name = GRIPPER_DEVICE_NAME
        env_creator.tool_name = GRIPPER_TOOL_NAME
        # The dual config already enables async_control, so the setters do not wait for the arms.
        hw_cfg = env_creator.config()
        # Drop the arms that are not connected.
        hw_cfg.robot_cfgs = {name: cfg for name, cfg in hw_cfg.robot_cfgs.items() if name in ROBOT_SN}
        assert hw_cfg.gripper_cfgs is not None
        hw_cfg.gripper_cfgs = {name: cfg for name, cfg in hw_cfg.gripper_cfgs.items() if name in ROBOT_SN}
        for robot_cfg in hw_cfg.robot_cfgs.values():
            # The robot's own Cartesian impedance controller tracks the streamed poses, no rcs IK involved. Its
            # motion generator limits how fast it follows the controller, raise them if the arm feels laggy.
            robot_cfg.control_mode = FlexivControlMode.CARTESIAN_IMPEDANCE
            robot_cfg.max_linear_velocity = 1.0
            robot_cfg.max_angular_velocity = 2.0
            robot_cfg.max_linear_acceleration = 4.0
            robot_cfg.max_angular_acceleration = 8.0
            # Reference posture for the null-space control, otherwise the posture at mode entry is used.
            robot_cfg.null_space_posture = robot_cfg.q_home
        hw_cfg.control_mode = config.operator_class.control_mode[0]
        hw_cfg.relative_to = config.operator_class.control_mode[1]
        hw_cfg.max_relative_movement = (0.5, np.deg2rad(90))
        hw_cfg.robot_to_shared_base_frame = {name: ROBOT_TO_SHARED_BASE_FRAME[name] for name in ROBOT_SN}
        hw_cfg.wrapper_cfg.binary_gripper = False
        hw_cfg.frequency = RECORD_FPS
        env_rel = env_creator.create_env(hw_cfg)
        operator = QuestOperator(config)
    else:
        scene = EmptyWorldRizon4SDuo()
        sim_cfg_data = scene.config()
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True, realtime=True, frequency=RECORD_FPS, max_convergence_steps=500
        )
        sim_cfg_data.control_mode = config.operator_class.control_mode[0]
        sim_cfg_data.relative_to = config.operator_class.control_mode[1]
        sim_cfg_data.max_relative_movement = (0.5, np.deg2rad(90))
        sim_cfg_data.robot_to_shared_base_frame = ROBOT_TO_SHARED_BASE_FRAME
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
