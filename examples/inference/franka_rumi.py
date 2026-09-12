import copy
import json
import logging
import threading
from dataclasses import asdict, dataclass, field
from pathlib import Path
from queue import Empty, Queue
from time import sleep
from typing import Any

from PIL import Image

from rcs.utils import SimpleFrameRate

import gymnasium as gym
import numpy as np
from rcs._core.common import BaseCameraConfig, RobotPlatform, GripperType
from rcs._core.sim import SimConfig
from rcs.camera.utils import capture_blank_camera_images
from rcs.envs.base import BlankCameraObservationWrapper, ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3Duo
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.envs.tasks import PickTaskConfig

import rcs
# from rcs_duobench.tasks.bin_sort import BinSortEnvConfig
from vlagents.client import RemoteAgent
from vlagents.policies import Act, Obs

logger = logging.getLogger(__name__)


GRIPPER_VERSION = "Robotiq2F85" # FrankaHand or Robotiq2F85
ROBOT2IP = {
    # "right": "192.168.102.1",
    "right": "192.168.102.1",
}
ROBOT2ID = {
    # "left": "0",
    "right": "0",
}


# ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_INSTANCE = RobotPlatform.HARDWARE

# set camera dict to none disable cameras
CAMERA_DICT = {
    "wrist": "230422272017",
}

INCLUDE_DEPTH = False

if GRIPPER_VERSION == "Robotiq2F85":
    FOLLOWER_GRIPPER_TYPE = GripperType("Robotiq2F85")
    DIGIT_DICT = { # Robotiq digits
        "digit_right_left": "D21154",
        "digit_right_right": "D21296",
    }
else:
    FOLLOWER_GRIPPER_TYPE = GripperType.FrankaHand
    DIGIT_DICT = { # Franka Hand digits
        "digit_right_left": "D21182",
        "digit_right_right": "D21193",
    }
# DIGIT_DICT = None 
ZED_CAMERA_DICT = None
INSTRUCTION = "pick up cube"
FPS = 30
CONTROL_MODE = ControlMode.JOINTS
RELATIVETO = RelativeTo.NONE
# RELATIVETO = RelativeTo.CONFIGURED_ORIGIN
RECORD_PATH = "BC_act"
MODEL = "lerobot"
IP = "localhost"
PORT = 20000
CONFIG_PATH = Path(__file__).with_suffix(".json")
MAX_REL_MOV_JOINTS = np.deg2rad(0.5)
MAX_REL_MOV_CART = (0.5, np.deg2rad(90))
ACTION_SPACES = ("joints", "xyzrpy", "delta_xyzrpy", "tquat", "delta_tquat")
ACTION_SPACE = "delta_xyzrpy"
INTEGRATE_DELTAS_FROM_COMMAND = True
# Set to True to close the binary gripper after every environment reset.
# The robot arm still returns to its configured home position.
START_GRIPPER_CLOSED = False

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    level=logging.INFO,
)


robot2world = {
    "right": rcs.common.Pose(
        translation=np.array([0, 0, 0]), rpy_vector=np.array([0, 0, 0])
    ),
}


@dataclass
class InferenceConfig:
    vlagents_host: str = IP
    vlagents_port: int = PORT
    vlagents_model: str = MODEL
    instruction: str = INSTRUCTION
    robot_keys: list[str] = field(default_factory=lambda: ["right"])
    jpeg_encoding: bool = True
    on_same_machine: bool = False
    fps: int = FPS
    record_path: str = RECORD_PATH
    n_action_steps: int | None = None
    max_rel_mov_joints: float = MAX_REL_MOV_JOINTS
    max_rel_mov_cart: tuple[float, float] = MAX_REL_MOV_CART
    action_space: str = ACTION_SPACE
    integrate_deltas_from_command: bool = INTEGRATE_DELTAS_FROM_COMMAND


def load_inference_config() -> InferenceConfig:
    if not CONFIG_PATH.exists():
        CONFIG_PATH.write_text(json.dumps(asdict(InferenceConfig()), indent=2) + "\n")
        return InferenceConfig()
    cfg = InferenceConfig(**json.loads(CONFIG_PATH.read_text()))
    if cfg.action_space not in ACTION_SPACES:
        raise ValueError(f"action_space must be one of {ACTION_SPACES}, got {cfg.action_space!r}")
    if cfg.integrate_deltas_from_command and cfg.action_space not in {"delta_xyzrpy", "delta_tquat"}:
        raise ValueError("integrate_deltas_from_command requires delta_xyzrpy or delta_tquat action_space")
    # Keep inference single-arm even if existing config stores two-arm keys.
    cfg.robot_keys = [key for key in cfg.robot_keys if key == "right"]
    if "right" not in cfg.robot_keys:
        cfg.robot_keys = ["right"]
    if len(cfg.robot_keys) != 1:
        logger.warning("Forcing single-arm inference mode: using robot key ['right']")
        cfg.robot_keys = ["right"]
    return cfg


def reset_env(env: gym.Env, cfg: InferenceConfig) -> tuple[dict, dict]:
    """Reset the environment and optionally command the gripper closed."""
    obs, info = env.reset()
    if not START_GRIPPER_CLOSED:
        return obs, info

    # For a binary gripper, 0 is closed. Reuse the reset joint positions so
    # this extra action does not move the arm away from its home position.
    action = {
        robot: {
            "joints": np.asarray(obs[robot]["joints"], dtype=np.float32),
            "gripper": np.array([0.0], dtype=np.float32),
        }
        for robot in cfg.robot_keys
    }
    obs, _, _, _, _ = env.step(action)
    return obs, info



class ModelInference:
    def __init__(self, env: gym.Env, cfg: InferenceConfig):
        self.env = env
        self.gripper_state = 1
        self._cfg = cfg
        self._episode_running = False
        self._command_queue: Queue[str] = Queue()
        self._shutdown_requested = threading.Event()
        self.remote_agent = RemoteAgent(
            cfg.vlagents_host, cfg.vlagents_port, cfg.vlagents_model, cfg.on_same_machine, cfg.jpeg_encoding
        )
        self.frame_rate = SimpleFrameRate(self._cfg.fps)
        self._action_buffer = []
        self._commanded_pose = None

    def _reset(self) -> tuple[dict, dict]:
        """Reset and initialize the commanded Cartesian reference from measured state."""
        obs, info = reset_env(self.env, self._cfg)
        if self._cfg.integrate_deltas_from_command:
            initial_tquat = np.asarray(obs["right"]["tquat"], dtype=np.float64).reshape(-1).copy()
            if initial_tquat.shape != (7,) or not np.isfinite(initial_tquat).all():
                raise ValueError(f"Invalid reset tquat for integrated replay: {initial_tquat}")
            self._commanded_pose = rcs.common.Pose(
                translation=initial_tquat[:3].reshape(3, 1),
                quaternion=initial_tquat[3:].reshape(4, 1),
            )
        else:
            self._commanded_pose = None
        return obs, info

    def submit_command(self, command: str) -> None:
        self._command_queue.put(command)

    def request_shutdown(self) -> None:
        self._shutdown_requested.set()

    def _drain_commands(self) -> tuple[bool, bool, bool, bool, bool]:
        start_requested = False
        record_requested = False
        success_requested = False
        stop_requested = False
        reload_requested = False

        while True:
            try:
                command = self._command_queue.get_nowait()
            except Empty:
                break

            if command == "e":
                start_requested = True
            elif command == "r":
                record_requested = True
            elif command == "s":
                success_requested = True
            elif command == "q":
                stop_requested = True
            elif command == "o":
                reload_requested = True

        return start_requested, record_requested, success_requested, stop_requested, reload_requested

    def obs_rcs2agents(self, obs: dict, info: dict | None = None) -> Obs:
        cameras = {}
        for frame in obs["frames"]:
            cameras[frame] = obs["frames"][frame]["rgb"]["data"]
            cameras[frame] = np.array(Image.fromarray(cameras[frame]).resize((224, 224), Image.Resampling.BILINEAR))

        state = []
        for robot in self._cfg.robot_keys:
            if robot not in obs:
                logger.warning("Observation missing robot %s; skipping", robot)
                continue
            # TODO: currently hardcoded for joints
            state.append(obs[robot]["joints"])
            state.append(obs[robot]["gripper"])
        return Obs(cameras=cameras, gripper=None, info=info, state=np.concatenate(state))

    def act(self, obs_dict) -> None:
        done = False
        if self._cfg.n_action_steps is None:
            return self.remote_agent.act(obs_dict)
        if len(self._action_buffer) == 0:
            action = self.remote_agent.act(obs_dict)
            selected_action = action.action[:self._cfg.n_action_steps]
            self._action_buffer = selected_action.tolist()
            done = action.done
            if RELATIVETO == RelativeTo.CONFIGURED_ORIGIN:
                for robot in self.env.get_wrapper_attr("envs"):
                    self.env.get_wrapper_attr("envs")[robot].get_wrapper_attr("set_origin_to_current")()
        act = self._action_buffer.pop(0)
        return Act(action=act, done=done)

    def action_agents2rcs(self, action: Act) -> dict[str, Any]:
        act = {}
        action_values = np.asarray(action.action, dtype=np.float32) if action.action is not None else np.array([])
        action_dim = {"joints": 8, "xyzrpy": 7, "delta_xyzrpy": 7, "tquat": 8, "delta_tquat": 8}[
            self._cfg.action_space
        ]
        for idx, robot in enumerate(self._cfg.robot_keys):
            start = idx * action_dim
            end = start + action_dim
            if end > len(action_values):
                logger.warning(
                    "Action vector too short for robot %s: expected %d values, got %d",
                    robot,
                    action_dim,
                    max(0, len(action_values) - start),
                )
                continue
            act[robot] = {}
            arm_action = action_values[start : end - 1]
            if self._cfg.action_space == "joints":
                act[robot]["joints"] = arm_action
            elif self._cfg.integrate_deltas_from_command:
                if self._commanded_pose is None:
                    raise RuntimeError("Commanded pose was not initialized after environment reset")
                if self._cfg.action_space == "delta_xyzrpy":
                    delta_rotation = rcs.common.Pose(rpy_vector=arm_action[3:].reshape(3, 1), translation=np.zeros((3, 1)))
                    next_rotation = delta_rotation * rcs.common.Pose(
                        quaternion=self._commanded_pose.rotation_q().reshape(4, 1)
                    )
                    self._commanded_pose = rcs.common.Pose(
                        translation=(
                            np.asarray(self._commanded_pose.translation()).reshape(-1) + arm_action[:3]
                        ).reshape(3, 1),
                        quaternion=next_rotation.rotation_q().reshape(4, 1),
                    )
                    act[robot]["xyzrpy"] = np.asarray(self._commanded_pose.xyzrpy()).reshape(-1)
                elif self._cfg.action_space == "delta_tquat":
                    delta_pose = rcs.common.Pose(
                        translation=arm_action[:3].reshape(3, 1),
                        quaternion=arm_action[3:].reshape(4, 1),
                    )
                    self._commanded_pose = delta_pose * self._commanded_pose
                    act[robot]["tquat"] = np.concatenate(
                        [
                            np.asarray(self._commanded_pose.translation()).reshape(-1),
                            np.asarray(self._commanded_pose.rotation_q()).reshape(-1),
                        ]
                    )
            elif self._cfg.action_space in {"xyzrpy", "delta_xyzrpy"}:
                act[robot]["xyzrpy"] = arm_action
            else:
                act[robot]["tquat"] = arm_action
            act[robot]["gripper"] = action_values[end - 1 : end]
        return act

    def loop(self):
        obs, _ = self._reset()
        obs_dict = self.obs_rcs2agents(obs)
        logger.info(
            "waiting for input: 'e' to start, 'r' to start and record, 's' for success and reset, 'q' to stop and reset, and 'o' to reload config"
        )

        while not self._shutdown_requested.is_set():
            start_requested, record_requested, success_requested, stop_requested, reload_requested = (
                self._drain_commands()
            )

            if reload_requested:
                self._cfg = load_inference_config()
                try:
                    self.remote_agent.reconnect(
                        host=self._cfg.vlagents_host,
                        port=self._cfg.vlagents_port,
                        model=self._cfg.vlagents_model,
                        on_same_machine=self._cfg.on_same_machine,
                        jpeg_encoding=self._cfg.jpeg_encoding,
                    )
                    logger.info(
                        "reloaded config from %s with host=%s port=%s model=%s",
                        CONFIG_PATH,
                        self._cfg.vlagents_host,
                        self._cfg.vlagents_port,
                        self._cfg.vlagents_model,
                    )
                except Exception:
                    logger.exception("failed to reconnect after reloading %s", CONFIG_PATH)
                if isinstance(self.env, StorageWrapper):
                    self.env.base_dir = self._cfg.record_path
                    self.env.set_instruction(self._cfg.instruction)
                obs, _ = self._reset()
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False

            if success_requested:
                if self._episode_running:
                    logger.info("marking episode successful and resetting environment")
                self.env.get_wrapper_attr("success")()
                obs, _ = self._reset()
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False

            if stop_requested:
                if self._episode_running:
                    logger.info("stopping episode and resetting environment")
                obs, _ = self._reset()
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False

            if not self._episode_running:
                try:
                    self.remote_agent.ensure_connected()
                except Exception:
                    sleep(0.5)
                    continue
                if start_requested or record_requested:
                    if isinstance(self.env, StorageWrapper):
                        self.env.set_instruction(self._cfg.instruction)
                        if record_requested:
                            self.env.start_record()
                    logger.info("starting episode%s", " with recording" if record_requested else "")
                    self.remote_agent.reset(copy.deepcopy(obs_dict), instruction=self._cfg.instruction)
                    self._episode_running = True
                else:
                    sleep(0.05)
                    continue

            action = self.act(copy.deepcopy(obs_dict))
            if action.done:
                logger.info("done issued by agent, resetting environment")
                obs, _ = reset_env(self.env, self._cfg)
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False
                continue
            a = self.action_agents2rcs(action)
            obs, _, _, _, info = self.env.step(a)
            # print(obs["left"]["joints"], obs["left"]["gripper"], obs["right"]["joints"], obs["right"]["gripper"])

            obs_dict = self.obs_rcs2agents(obs)

            if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
                self.frame_rate()


def command_loop(controller: ModelInference) -> None:
    prompt = "Command [e=start, r=record, s=success/reset, q=stop/reset, o=reload, x=exit]: "
    while True:
        try:
            command = input(prompt).strip().lower()
        except EOFError:
            command = "x"
        except KeyboardInterrupt:
            print()
            command = "x"

        if not command:
            continue
        if command == "x":
            controller.request_shutdown()
            return
        if command in {"e", "r", "s", "q", "o"}:
            controller.submit_command(command)
            continue
        logger.info("unknown command %r", command)


def get_env(cfg: InferenceConfig) -> gym.Env:
    blank_camera_dict: dict[str, np.ndarray] = {}
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_fr3.configs import SingleArmFR3MultiHardwareEnv
        from rcs_fr3.creators import HardwareCameraCreatorConfig

        env_creator = SingleArmFR3MultiHardwareEnv()
        env_creator.ip = ROBOT2IP["right"]
        hw_cfg = env_creator.config(grippertype=FOLLOWER_GRIPPER_TYPE, robot_ip=ROBOT2IP["right"])
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
        if cfg.action_space == "joints":
            hw_cfg.control_mode = ControlMode.JOINTS
            hw_cfg.relative_to = RelativeTo.NONE
            hw_cfg.max_relative_movement = cfg.max_rel_mov_joints
        elif cfg.action_space in {"xyzrpy", "delta_xyzrpy"}:
            hw_cfg.control_mode = ControlMode.CARTESIAN_TRPY
            hw_cfg.relative_to = (
                RelativeTo.NONE
                if cfg.action_space == "xyzrpy"
                or cfg.integrate_deltas_from_command and cfg.action_space == "delta_xyzrpy"
                else RelativeTo.LAST_STEP
            )
            hw_cfg.max_relative_movement = cfg.max_rel_mov_cart
        else:
            hw_cfg.control_mode = ControlMode.CARTESIAN_TQuat
            hw_cfg.relative_to = (
                RelativeTo.NONE
                if cfg.action_space == "tquat"
                or cfg.integrate_deltas_from_command and cfg.action_space == "delta_tquat"
                else RelativeTo.LAST_STEP
            )
            hw_cfg.max_relative_movement = cfg.max_rel_mov_cart
        if cfg.integrate_deltas_from_command and cfg.action_space in {"delta_xyzrpy", "delta_tquat"}:
            # Avoid LimitedAbsoluteAction recomputing an integrated target
            # from the measured pose.
            hw_cfg.max_relative_movement = None
        hw_cfg.wrapper_cfg.include_depth = INCLUDE_DEPTH
        hw_cfg.robot_to_shared_base_frame = robot2world
        hw_cfg.robot_cfgs["right"].ignore_realtime = True
        hw_cfg.robot_cfgs["right"].speed_factor = 0.1

        # Gains used for USBC: x10
        # Gains for Box, wiping vase/chalk, screw: x 7
        hw_cfg.robot_cfgs["right"].joint_controller_Kp = 20*np.array([24,24,24,24,10,6,3])
        hw_cfg.robot_cfgs["right"].joint_controller_Kd = 2*np.sqrt(hw_cfg.robot_cfgs["right"].joint_controller_Kp)
        hw_cfg.robot_cfgs["right"].joint_controller_torque_limits = np.array([12.0, 12.0, 12.0, 10.0, 5.0, 4.0, 3.0])
        # 2 *  np.asarray([200, 200, 75]) # board task
        hw_cfg.robot_cfgs["right"].osc_Kp_p = 2 *  np.asarray([150, 150, 150])#np.asarray([150, 150, 150])
        hw_cfg.robot_cfgs["right"].osc_Kp_r = 1 *  np.asarray([250, 250, 250])
        hw_cfg.robot_cfgs["right"].osc_torque_limits = np.asarray([12.0, 12.0, 12.0, 10.0, 5.0, 4.0, 3.0])
        # q_home for gear
        # hw_cfg.robot_cfgs["right"].q_home = np.array([ 0.12982505,  0.22033154, -0.19202998, -2.25742164,  0.55279994, 2.9276454 , -0.6510375 ])
        # q_home for board
        hw_cfg.robot_cfgs["right"].q_home = np.array([ 0.10142963,  0.02549116, -0.15556482, -2.34184856,  0.16230607, 2.71370075, -0.24183754])
        hw_cfg.wrapper_cfg.binary_gripper = False
        env_rel = env_creator.create_env(hw_cfg)
        if DIGIT_DICT is not None:
            camera_set = env_rel.get_wrapper_attr("camera_set")
            blank_camera_dict = capture_blank_camera_images(camera_set, DIGIT_DICT)
    else:
        # FR3

        # scene = BinSortEnvConfig()
        sim_cfg_data = scene.config()
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True, realtime=False, frequency=cfg.fps, max_convergence_steps=500
        )
        sim_cfg_data.wrapper_cfg.include_depth = INCLUDE_DEPTH
        if cfg.action_space == "joints":
            sim_cfg_data.control_mode = ControlMode.JOINTS
            sim_cfg_data.relative_to = RelativeTo.NONE
            sim_cfg_data.max_relative_movement = cfg.max_rel_mov_joints
        elif cfg.action_space in {"xyzrpy", "delta_xyzrpy"}:
            sim_cfg_data.control_mode = ControlMode.CARTESIAN_TRPY
            sim_cfg_data.relative_to = (
                RelativeTo.NONE
                if cfg.action_space == "xyzrpy"
                or cfg.integrate_deltas_from_command and cfg.action_space == "delta_xyzrpy"
                else RelativeTo.LAST_STEP
            )
            sim_cfg_data.max_relative_movement = cfg.max_rel_mov_cart
        else:
            sim_cfg_data.control_mode = ControlMode.CARTESIAN_TQuat
            sim_cfg_data.relative_to = (
                RelativeTo.NONE
                if cfg.action_space == "tquat"
                or cfg.integrate_deltas_from_command and cfg.action_space == "delta_tquat"
                else RelativeTo.LAST_STEP
            )
            sim_cfg_data.max_relative_movement = cfg.max_rel_mov_cart
        if cfg.integrate_deltas_from_command and cfg.action_space in {"delta_xyzrpy", "delta_tquat"}:
            sim_cfg_data.max_relative_movement = None
        sim_cfg_data.wrapper_cfg.binary_gripper = False


        # if sim_cfg_data.root_frame_objects is None:
        #     sim_cfg_data.root_frame_objects = {}
        # sim_cfg_data.task_cfg = PickTaskConfig(robot_name="right")

        env_rel = scene.create_env(sim_cfg_data)

    if blank_camera_dict:
        env_rel = BlankCameraObservationWrapper(env_rel, blank_camera_dict)

    return StorageWrapper(
        env_rel,
        cfg.record_path,
        cfg.instruction,
        batch_size=32,
        max_rows_per_group=2,
        max_rows_per_file=10,
        allow_wrapper_instruction=False
    )


def main():
    cfg = load_inference_config()
    env_rel = get_env(cfg)

    # Path(VIDEO_PATH).mkdir(parents=True, exist_ok=True)
    # timestamp = str(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    # camera_set = env_rel.get_wrapper_attr("camera_set")
    # camera_set.record_video(Path(VIDEO_PATH), timestamp)

    # env = RHCWrapper(env, exec_horizon=1)

    controller = ModelInference(env_rel, cfg)
    with env_rel:
        worker = threading.Thread(target=controller.loop, name="model-inference", daemon=True)
        worker.start()
        command_loop(controller)
        worker.join()


if __name__ == "__main__":
    main()
