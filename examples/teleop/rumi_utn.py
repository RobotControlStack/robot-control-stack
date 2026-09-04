import logging
import time
from dataclasses import dataclass

import numpy as np
from rcs._core.common import BaseCameraConfig, RobotPlatform, GripperType
from rcs._core.sim import SimConfig
from rcs.camera.utils import capture_blank_camera_images
from rcs.envs.base import BlankCameraObservationWrapper, CameraSetWrapper, ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3Duo, EmptyWorldFR3
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.envs.tasks import PickTaskConfig
from rcs.operator.gello import GelloConfig, GelloOperator
from rcs.operator.interface import BaseOperator, TeleopCommands, TeleopLoop
from rcs.operator.pedals import FootPedal
from rcs.operator.quest import QuestConfig, QuestOperator
from simpub.sim.mj_publisher import MujocoPublisher
from rcs_rumi import RumiFTRCSWrapper, RumiMode

import rcs

logger = logging.getLogger(__name__)


PEDAL_KEY_A = "KEY_A"
PEDAL_KEY_C = "KEY_C"
PYZLC_FAILED_NODE_RETRY_SECONDS = 10.0


def install_pyzlc_discovery_diagnostics() -> None:
    """Rate-limit failed IRIS node-info lookups and identify the failing peer.

    pyzlc normally retries ``get_node_info`` on every multicast heartbeat when
    the peer responds with an error status. Controller topics use separate
    sockets, so a stale or incompatible discovery peer can fail here while the
    Quest controller stream continues to work normally.
    """
    import pyzlc
    from pyzlc.nodes.nodes_info_manager import NodesInfoManager

    patch_marker = "_rumi_node_info_diagnostics_installed"
    if getattr(NodesInfoManager, patch_marker, False):
        return

    original_handle_heartbeat = NodesInfoManager.handle_heartbeat_async

    async def handle_heartbeat_with_backoff(self, heartbeat_message, node_ip):
        failure_state = getattr(self, "_rumi_node_info_failures", None)
        if failure_state is None:
            failure_state = {}
            self._rumi_node_info_failures = failure_state

        node_id = heartbeat_message.node_id
        now = time.monotonic()
        retry_at, failure_count = failure_state.get(node_id, (0.0, 0))
        if now < retry_at:
            # Preserve liveness for an already-known Quest node whose metadata
            # refresh failed; only the refresh is being rate-limited.
            if node_id in self.nodes_info:
                self.nodes_heartbeat[node_id] = now
            return

        await original_handle_heartbeat(self, heartbeat_message, node_ip)

        if self.check_info(node_id, heartbeat_message.info_id):
            if failure_count:
                pyzlc.info(
                    "IRIS discovery recovered for peer %s at %s:%s.",
                    node_id,
                    node_ip,
                    heartbeat_message.service_port,
                )
            failure_state.pop(node_id, None)
            return

        # send_request() converts error statuses and timeouts to None rather
        # than raising, so stock pyzlc does not add this peer to its suppression
        # set. Remove its incomplete heartbeat entry and apply a finite backoff
        # so discovery can recover without burdening the teleoperation process.
        self.unreplyed_heartbeats.discard(node_id)
        if node_id not in self.nodes_info:
            self.nodes_heartbeat.pop(node_id, None)
        failure_count += 1
        failure_state[node_id] = (now + PYZLC_FAILED_NODE_RETRY_SECONDS, failure_count)

        version = ".".join(str(part) for part in heartbeat_message.zlc_version)
        if failure_count == 1:
            pyzlc.warning(
                "IRIS discovery peer node_id=%s at %s:%s (protocol %s, info_id=%s) "
                "did not return usable metadata from get_node_info. This lookup is separate from controller "
                "pose streaming, so the Quest app may continue working. The peer may be stale or protocol-incompatible. "
                "Further lookups for this peer are limited to once every %.1f seconds to avoid teleoperation lag.",
                node_id,
                node_ip,
                heartbeat_message.service_port,
                version,
                heartbeat_message.info_id,
                PYZLC_FAILED_NODE_RETRY_SECONDS,
            )
        else:
            pyzlc.debug(
                "IRIS discovery retry %s failed for peer %s at %s:%s; next retry in %.1f seconds.",
                failure_count,
                node_id,
                node_ip,
                heartbeat_message.service_port,
                PYZLC_FAILED_NODE_RETRY_SECONDS,
            )

    NodesInfoManager.handle_heartbeat_async = handle_heartbeat_with_backoff
    setattr(NodesInfoManager, patch_marker, True)


@dataclass
class PedalEpisodeState:
    """Translate pedal edges into episode commands; pedal B is unbound."""

    active: bool = False
    a_was_pressed: bool = False
    c_was_pressed: bool = False

    def update_commands(self, pedal: FootPedal, commands: TeleopCommands) -> TeleopCommands:
        # Episode lifecycle is pedal-only in this script. Discard the Quest
        # A/B/Y commands before applying the pedal mapping.
        commands.record = False
        commands.success = False
        commands.failure = False

        a_is_pressed = pedal.get_key_state(PEDAL_KEY_A)
        c_is_pressed = pedal.get_key_state(PEDAL_KEY_C)
        a_pressed = a_is_pressed and not self.a_was_pressed
        c_pressed = c_is_pressed and not self.c_was_pressed
        self.a_was_pressed = a_is_pressed
        self.c_was_pressed = c_is_pressed

        if c_pressed and self.active:
            commands.success = True
            self.active = False
        elif a_pressed:
            if self.active:
                commands.failure = True
                self.active = False
            else:
                commands.record = True
                self.active = True
        return commands


class FootPedalEpisodeOperator(BaseOperator):
    """Delegate robot control while sourcing episode commands from a pedal."""

    def __init__(self, operator: BaseOperator, pedal: FootPedal):
        super().__init__(operator.config, operator.sim)
        self.operator = operator
        self.pedal = pedal
        self.episode_state = PedalEpisodeState()
        self.control_mode = operator.control_mode
        self.controller_names = operator.controller_names

    def start(self):
        self.operator.start()

    def join(self, timeout: float | None = None):
        self.operator.join(timeout)

    def close(self):
        self.operator.close()

    def reset_operator_state(self):
        self.operator.reset_operator_state()

    def consume_commands(self) -> TeleopCommands:
        return self.episode_state.update_commands(self.pedal, self.operator.consume_commands())

    def consume_action(self):
        return self.operator.consume_action()

    def set_camera(self, observation):
        self.operator.set_camera(observation)


ROBOT2IP = {
    # "right": "192.168.102.1",
    "right": "192.168.101.1",
}
ROBOT2ID = {
    # "left": "0",
    "right": "0",
}


ROBOT_INSTANCE = RobotPlatform.SIMULATION
# ROBOT_INSTANCE = RobotPlatform.HARDWARE

RECORD_FPS = 30
# set camera dict to none disable cameras
# "left_wrist": "230422272017",
# "right_wrist": "230422271040",

CAMERA_DICT = {
    "wrist": "230422272017",
}
# CAMERA_DICT = None
ZED_CAMERA_DICT = None
MQ3_ADDR = "10.42.0.1"  # Jin: IPv4 address of the wifi you are connected, join the same network with Quest
# Jin: After configuring this, run quest_align_frame.py, THEN start the IRIS app. If you see the controller pose in print, youre good
# Jin: After that, exit quest_align_frame.py (ctrl c a few times), then run franka.py; then you are ready to teleop
INCLUDE_DEPTH = False

# DIGIT_DICT = {"digit_right_left": "D21154", "digit_right_right": "D21296"}
DIGIT_DICT = None


DATASET_PATH = "rumi_debug"
INSTRUCTION = "pick up cube"
RECORD_FPS = 30

robot2world = {
    # "right": rcs.common.Pose(
    #     translation=np.array([0, 0, 0]), rpy_vector=np.array([0.89360858, -0.17453293, 0.46425758])
    # ),
    # "left": rcs.common.Pose(
    # translation=np.array([0, 0, 0]), rpy_vector=np.array([-0.89360858, -0.17453293, -0.46425758])
    # ),
    "right": rcs.common.Pose(translation=np.array([0, 0, 0]), rpy_vector=np.array([0, 0, 0])),
}

config: QuestConfig | GelloConfig

import scipy.spatial.transform as spt
quat_offset = spt.Rotation.from_quat([0.7071068, 0, 0.7071068, 0])
quat_finetune = spt.Rotation.from_euler("xyz", [0,-1.5,4], degrees=True)
quat_final = quat_finetune * quat_offset
quat_final = quat_final.as_quat()

config = QuestConfig(
    mq3_addr=MQ3_ADDR,
    simulation=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
    switched_left_right=False,
    display_cameras=False,
    umi_mode=True,
    umi_mode_tool_offset={
        "right": rcs.common.Pose(translation=np.array([0.20, -0.007, -0.05]), quaternion=quat_final)
    },
    absolute_tracking=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
)
# config = GelloConfig(
#     arms={
#         "right": GelloArmConfig(com_port="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_E505008B503059384C2E3120FF07332D-if00"),
#         "left": GelloArmConfig(com_port="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_ABA78B05503059384C2E3120FF062F26-if00"),
#     },
#     simulation=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
# )


def get_env():
    blank_camera_dict: dict[str, np.ndarray] = {}
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        raise NotImplementedError("Hardware mode is not implemented yet.")
        from rcs_fr3.configs import SingleArmFR3MultiHardwareEnv
        from rcs_fr3.creators import HardwareCameraCreatorConfig

        env_creator = SingleArmFR3MultiHardwareEnv()
        env_creator.ip = ROBOT2IP["right"]
        hw_cfg = env_creator.config(grippertype=GripperType("Robotiq2F85"), robot_ip=ROBOT2IP["right"])
        camera_cfgs: dict[str, HardwareCameraCreatorConfig] = {}
        if CAMERA_DICT is not None:
            try:
                from rcs_realsense.utils import reset_cameras

                reset_cameras()
            except Exception as e:
                print("Error occurred while resetting cameras: %s", e)
                print("Assuming realsense is not being used, continuing.")
            camera_cfgs["realsense"] = HardwareCameraCreatorConfig(
                camera_type_id="realsense",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=640,
                        resolution_height=480,
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
                        resolution_width=640,
                        resolution_height=480,
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
        hw_cfg.wrapper_cfg.binary_gripper = False
        hw_cfg.max_relative_movement = (
            0.5 if config.operator_class.control_mode[0] == ControlMode.JOINTS else (0.5, np.deg2rad(90))
        )
        hw_cfg.relative_to = config.operator_class.control_mode[1]
        hw_cfg.robot_to_shared_base_frame = robot2world
        env_rel = env_creator.create_env(hw_cfg)
        if DIGIT_DICT is not None:
            camera_set = env_rel.get_wrapper_attr("camera_set")
            blank_camera_dict = capture_blank_camera_images(camera_set, DIGIT_DICT)
        operator = GelloOperator(config) if isinstance(config, GelloConfig) else QuestOperator(config)
    else:
        # FR3

        scene = EmptyWorldFR3()
        sim_cfg_data = scene.config()
        sim_cfg_data.gripper_cfgs['right'] = rcs._core.sim.SimGripperConfig(
            epsilon_inner=0.005,
            epsilon_outer=0.005,
            seconds_between_callbacks=0.1,
            ignored_collision_geoms=[],
            collision_geoms=[],
            collision_geoms_fingers=[],
            joints=["right_driver_joint", "left_driver_joint"],
            max_joint_width=0.005,
            min_joint_width=1.0,
            actuator="fingers_actuator",
            max_actuator_width=0,
            min_actuator_width=255,
            gripper_type=GripperType("Robotiq2F85"),
        )
        sim_cfg_data.robot_cfgs['right'].tcp_offset=rcs.GRIPPER_TCP_OFFSETS[rcs.common.GripperType("Robotiq2F85")]
        # q_home measured based on ergonomics, adjust depending on setup
        sim_cfg_data.robot_cfgs['right'].q_home=np.array([-0.87038961,-0.22665566,  1.52779737, -2.30577027, -0.114296,    2.53977886,   0.72123607])
        sim_cfg_data.gripper_offsets['right'] = rcs.GRIPPER_MOUNT_OFFSETS[rcs.common.GripperType("Robotiq2F85")]
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True, realtime=True, frequency=RECORD_FPS, max_convergence_steps=500
        )
        # Quest UMI mode emits poses directly in the simulation/shared frame.
        sim_cfg_data.relative_to = RelativeTo.NONE
        sim_cfg_data.max_relative_movement = None
        sim_cfg_data.wrapper_cfg.include_depth = INCLUDE_DEPTH
        sim_cfg_data.wrapper_cfg.binary_gripper = False

        # Build the physical sensor stack independently of the simulated robot.
        from rcs_fr3.creators import HardwareCameraCreatorConfig, _create_hardware_camera_set

        camera_cfgs: dict[str, HardwareCameraCreatorConfig] = {}
        if CAMERA_DICT is not None:
            try:
                from rcs_realsense.utils import reset_cameras

                reset_cameras()
            except Exception as e:
                logger.warning("Could not reset RealSense cameras; continuing without a reset: %s", e)
            camera_cfgs["realsense"] = HardwareCameraCreatorConfig(
                camera_type_id="realsense",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=640,
                        resolution_height=480,
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
                        resolution_width=640,
                        resolution_height=480,
                        frame_rate=30,
                    )
                    for name, identifier in ZED_CAMERA_DICT.items()
                },
                kwargs={
                    "enable_depth": INCLUDE_DEPTH,
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

        hardware_camera_set = _create_hardware_camera_set(camera_cfgs or None)
        if hardware_camera_set is not None:
            # CameraSetWrapper owns the complete camera observation payload. Disable
            # MuJoCo cameras to avoid duplicate names and an inconsistent space.
            sim_cfg_data.camera_cfgs = None
            sim_cfg_data.camera_adds = None

        if sim_cfg_data.root_frame_objects is None:
            sim_cfg_data.root_frame_objects = {}
        # cfg.root_frame_objects["green_cube"] = (rcs.OBJECT_PATHS["green_cube"], Pose(translation=[0.5, 0, 0.5], quaternion=[0, 0, 0, 1]))
        sim_cfg_data.task_cfg = PickTaskConfig(robot_name="right")
        env_rel = scene.create_env(sim_cfg_data)

        if hardware_camera_set is not None:
            try:
                hardware_camera_set.start()
                hardware_camera_set.wait_for_frames()
            except Exception:
                hardware_camera_set.close()
                env_rel.close()
                raise
            env_rel = CameraSetWrapper(env_rel, hardware_camera_set, include_depth=INCLUDE_DEPTH)
            if DIGIT_DICT is not None:
                blank_camera_dict = capture_blank_camera_images(hardware_camera_set, DIGIT_DICT)

        sim = env_rel.get_wrapper_attr("sim")
        MujocoPublisher(sim.model, sim.data, MQ3_ADDR, visible_geoms_groups=list(range(1, 3)))
        operator = GelloOperator(config, sim) if isinstance(config, GelloConfig) else QuestOperator(config, sim)

    if blank_camera_dict:
        env_rel = BlankCameraObservationWrapper(env_rel, blank_camera_dict)

    env_rel = RumiFTRCSWrapper(env_rel, read_timeout=0.1, sim_robot_gripper_prefix="gripper", mode=RumiMode.WILD)
    env_rel = StorageWrapper(
        env_rel,
        DATASET_PATH,
        INSTRUCTION,
        batch_size=32,
        max_rows_per_group=100,
        max_rows_per_file=1000,
    )
    return env_rel, operator


def main():
    install_pyzlc_discovery_diagnostics()
    env_rel, operator = get_env()
    env_rel.reset()
    pedal = FootPedal("FootSwitch Keyboard")
    try:
        pedal_operator = FootPedalEpisodeOperator(operator, pedal)
        tele = TeleopLoop(
            env_rel,
            pedal_operator,
            env_frequency=RECORD_FPS,
            robot_platform=ROBOT_INSTANCE,
            catch_to_teleop=ROBOT_INSTANCE == RobotPlatform.SIMULATION,
        )
        with env_rel, tele:  # type: ignore
            tele.environment_step_loop()
    finally:
        pedal.close()


if __name__ == "__main__":
    main()
