import logging

import numpy as np
from rcs._core.common import BaseCameraConfig, RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3Duo
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.envs.tasks import PickTaskConfig
from rcs.operator.gello import GelloConfig, GelloOperator
from rcs.operator.interface import TeleopLoop
from rcs.operator.quest import QuestConfig, QuestOperator
from simpub.sim.mj_publisher import MujocoPublisher

import rcs

logger = logging.getLogger(__name__)


ROBOT2IP = {
    "right": "192.168.102.1",
    "left": "192.168.101.1",
}
ROBOT2ID = {
    "left": "1",
    "right": "0",
}


ROBOT_INSTANCE = RobotPlatform.SIMULATION
# ROBOT_INSTANCE = RobotPlatform.HARDWARE

RECORD_FPS = 30
# set camera dict to none disable cameras
# CAMERA_DICT = {
#     "left_wrist": "230422272017",
#     "right_wrist": "230422271040",
#     "side": "243522070385",
#     "bird_eye": "243522070364",
# }
CAMERA_DICT = None
ZED_CAMERA_DICT = {
    "zed": "19928076",
}
MQ3_ADDR = "10.42.0.1"
INCLUDE_DEPTH = False

# DIGIT_DICT = {
#     "digit_right_left": "D21182",
#     "digit_right_right": "D21193"
# }
DIGIT_DICT = None


DATASET_PATH = "test_iris"
INSTRUCTION = "pick up cube"
RECORD_FPS = 30

robot2world = {
    "right": rcs.common.Pose(
        translation=np.array([0, 0, 0]), rpy_vector=np.array([0.89360858, -0.17453293, 0.46425758])
    ),
    "left": rcs.common.Pose(
        translation=np.array([0, 0, 0]), rpy_vector=np.array([-0.89360858, -0.17453293, -0.46425758])
    ),
}

config: QuestConfig | GelloConfig
config = QuestConfig(
    mq3_addr=MQ3_ADDR,
    simulation=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
    switched_left_right=False,
    display_cameras=False,
)
# config = GelloConfig(
#     arms={
#         "right": GelloArmConfig(com_port="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_E505008B503059384C2E3120FF07332D-if00"),
#         "left": GelloArmConfig(com_port="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_ABA78B05503059384C2E3120FF062F26-if00"),
#     },
#     simulation=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
# )


def get_env():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_fr3.configs import FrankaDuoEnv
        from rcs_fr3.creators import HardwareCameraCreatorConfig

        env_creator = FrankaDuoEnv()
        env_creator.left_ip = ROBOT2IP["left"]
        env_creator.right_ip = ROBOT2IP["right"]
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
        if ZED_CAMERA_DICT is not None:
            camera_cfgs["zed"] = HardwareCameraCreatorConfig(
                camera_type_id="zed",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=1280,
                        resolution_height=720,
                        frame_rate=30,
                    )
                    for name, identifier in ZED_CAMERA_DICT.items()
                },
                kwargs={
                    "enable_depth": False,
                    "enable_imu": False,
                },
            )
        if DIGIT_DICT is not None:
            camera_cfgs["digit"] = HardwareCameraCreatorConfig(
                camera_type_id="digit",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=320,
                        resolution_height=240,
                        frame_rate=30,
                    )
                    for name, identifier in DIGIT_DICT.items()
                },
            )
        hw_cfg.camera_cfgs = camera_cfgs or None
        hw_cfg.control_mode = config.operator_class.control_mode[0]
        hw_cfg.wrapper_cfg.include_depth = INCLUDE_DEPTH
        hw_cfg.max_relative_movement = (
            0.5 if config.operator_class.control_mode[0] == ControlMode.JOINTS else (0.5, np.deg2rad(90))
        )
        hw_cfg.relative_to = config.operator_class.control_mode[1]
        hw_cfg.robot_to_shared_base_frame = robot2world
        env_rel = env_creator.create_env(hw_cfg)
        operator = GelloOperator(config) if isinstance(config, GelloConfig) else QuestOperator(config)
    else:
        # FR3

        scene = EmptyWorldFR3Duo()
        sim_cfg_data = scene.config()
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True, realtime=True, frequency=RECORD_FPS, max_convergence_steps=500
        )
        sim_cfg_data.relative_to = RelativeTo.CONFIGURED_ORIGIN
        sim_cfg_data.wrapper_cfg.include_depth = INCLUDE_DEPTH
        if sim_cfg_data.root_frame_objects is None:
            sim_cfg_data.root_frame_objects = {}
        # cfg.root_frame_objects["green_cube"] = (rcs.OBJECT_PATHS["green_cube"], Pose(translation=[0.5, 0, 0.5], quaternion=[0, 0, 0, 1]))
        sim_cfg_data.task_cfg = PickTaskConfig(robot_name="right")

        env_rel = scene.create_env(sim_cfg_data)

        sim = env_rel.get_wrapper_attr("sim")
        MujocoPublisher(sim.model, sim.data, MQ3_ADDR, visible_geoms_groups=list(range(1, 3)))
        operator = GelloOperator(config, sim) if isinstance(config, GelloConfig) else QuestOperator(config, sim)

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
