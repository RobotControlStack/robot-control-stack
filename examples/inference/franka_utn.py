import copy
import datetime
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

visiononly = False

logger = logging.getLogger(__name__)


GRIPPER_VERSION = "FrankaHand" # FrankaHand or Robotiq2F85
ROBOT2IP = {
    # "right": "192.168.102.1",
    "right": "192.168.101.1",
}
ROBOT2ID = {
    # "left": "0",
    "right": "0",
}


# ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_INSTANCE = RobotPlatform.HARDWARE

# set camera dict to none disable cameras
CAMERA_DICT = {
    "wrist": "230422271040",
    "side": "243122074917",
}

# Frames matching these suffixes are NOT forwarded to the policy. The
# BlankCameraObservationWrapper adds a "<camera>_blank" reference frame for
# every DIGIT camera; the policy was not trained on those background images,
# so drop them while still forwarding the live tactile frames.
AGENT_CAMERA_EXCLUDE_SUFFIXES = ("_blank",)

ZED_CAMERA_DICT = None

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


#if visiononly:
 #   DIGIT_DICT = None

INSTRUCTION = "pick up cube"
FPS = 30
CONTROL_MODE = ControlMode.JOINTS
RELATIVETO = RelativeTo.NONE
# RELATIVETO = RelativeTo.CONFIGURED_ORIGIN
RECORD_PATH = "recorded_inference_recordings"
VIDEO_PATH = "recorded_inference_videos"
MODEL = "lerobot"
IP = "localhost"
PORT = 20000
CONFIG_PATH = Path(__file__).with_suffix(".json")
MAX_REL_MOV_JOINTS = np.deg2rad(0.5)
MAX_REL_MOV_CART = (0.5, np.deg2rad(90))
# Set to True to close the binary gripper after every environment reset.
# The robot arm still returns to its configured home position.
START_GRIPPER_CLOSED = False

# Constant mechanical offset on a single joint (mounting mistake). The offset
# is ADDED to the action sent to the robot and SUBTRACTED from the observed
# joint before it is passed to the policy, so the closed loop stays consistent.
# JOINT_OFFSET_INDEX is a 0-based index into the 7-DOF joint vector; Franka
# "joint 6" (1-based) corresponds to index 5. Set JOINT_OFFSET_RAD to 0 to
# disable.
JOINT_OFFSET_INDEX = 6
JOINT_OFFSET_RAD = np.deg2rad(45.0) * 0

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


def load_inference_config() -> InferenceConfig:
    if not CONFIG_PATH.exists():
        CONFIG_PATH.write_text(json.dumps(asdict(InferenceConfig()), indent=2) + "\n")
        return InferenceConfig()
    cfg = InferenceConfig(**json.loads(CONFIG_PATH.read_text()))
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
            if frame.endswith(AGENT_CAMERA_EXCLUDE_SUFFIXES):
                # Skip the blank "background" reference frames the policy was
                # not trained on, while keeping the live tactile frames.
                continue
            cameras[frame] = obs["frames"][frame]["rgb"]["data"]
            cameras[frame] = np.array(Image.fromarray(cameras[frame]).resize((224, 224), Image.Resampling.BILINEAR))

        state = []
        for robot in self._cfg.robot_keys:
            if robot not in obs:
                logger.warning("Observation missing robot %s; skipping", robot)
                continue
            # TODO: currently hardcoded for joints
            joints = np.asarray(obs[robot]["joints"], dtype=np.float32).copy()
            if JOINT_OFFSET_RAD and JOINT_OFFSET_INDEX < len(joints):
                # Undo the mechanical joint offset so the policy sees the
                # joint values it was trained on.
                joints[JOINT_OFFSET_INDEX] -= JOINT_OFFSET_RAD
            state.append(joints)
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
        for idx, robot in enumerate(self._cfg.robot_keys):
            # TODO: this is currently hard coded for franka joints
            start = idx * 8
            end = start + 8
            if end > len(action_values):
                logger.warning(
                    "Action vector too short for robot %s: expected 8 values, got %d",
                    robot,
                    max(0, len(action_values) - start),
                )
                continue
            act[robot] = {}
            act[robot]["joints"] = action_values[start:end - 1]
            act[robot]["gripper"] = action_values[end - 1 : end]
            if JOINT_OFFSET_RAD and JOINT_OFFSET_INDEX < len(act[robot]["joints"]):
                # Compensate the mechanical joint offset when commanding the robot.
                act[robot]["joints"] = act[robot]["joints"].copy()
                act[robot]["joints"][JOINT_OFFSET_INDEX] += JOINT_OFFSET_RAD
            #print("-------------", act[robot]["joints"], act[robot]["gripper"])
        return act

    def loop(self):
        obs, _ = reset_env(self.env, self._cfg)
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
                obs, _ = reset_env(self.env, self._cfg)
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False

            if success_requested:
                if self._episode_running:
                    logger.info("marking episode successful and resetting environment")
                self.env.get_wrapper_attr("success")()
                obs, _ = reset_env(self.env, self._cfg)
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False

            if stop_requested:
                if self._episode_running:
                    logger.info("stopping episode and resetting environment")
                obs, _ = reset_env(self.env, self._cfg)
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
                        # DIGIT QVGA frames come back portrait: shape (320, 240, 3).
                        # The width/height must match (frame = (height, width)) or
                        # the video writer silently drops every frame.
                        resolution_width=240,
                        resolution_height=320,
                        frame_rate=30,
                    )
                    for name, identifier in DIGIT_DICT.items()
                },
            )
        hw_cfg.camera_cfgs = camera_cfgs or None
        hw_cfg.control_mode = CONTROL_MODE
        hw_cfg.wrapper_cfg.include_depth = INCLUDE_DEPTH
        hw_cfg.max_relative_movement = cfg.max_rel_mov_joints if CONTROL_MODE == ControlMode.JOINTS else cfg.max_rel_mov_cart
        hw_cfg.relative_to = RELATIVETO
        hw_cfg.robot_to_shared_base_frame = robot2world
        hw_cfg.robot_cfgs["right"].ignore_realtime = True
        hw_cfg.robot_cfgs["right"].speed_factor = 0.1

        # Gains used for USBC
        hw_cfg.robot_cfgs["right"].joint_controller_Kp = np.array([100., 100., 100., 100., 75., 75., 30.])
        hw_cfg.robot_cfgs["right"].joint_controller_Kp = 1 * hw_cfg.robot_cfgs["right"].joint_controller_Kp
        hw_cfg.robot_cfgs["right"].joint_controller_Kd = 1 * np.sqrt(hw_cfg.robot_cfgs["right"].joint_controller_Kp)

        # q_home for the insertion_only case
        if START_GRIPPER_CLOSED:
            hw_cfg.robot_cfgs["right"].q_home = np.array([ 0.03649692, 0.29955739, -0.03282878, -2.14812803, -0.04740378,  2.49525499, 0.163381  ])

        hw_cfg.wrapper_cfg.binary_gripper = True
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
        sim_cfg_data.control_mode = ControlMode.JOINTS
        sim_cfg_data.relative_to = RELATIVETO
        sim_cfg_data.wrapper_cfg.binary_gripper = True
        sim_cfg_data.max_relative_movement = cfg.max_rel_mov_joints if CONTROL_MODE == ControlMode.JOINTS else cfg.max_rel_mov_cart


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
    reset_env(env_rel, cfg)
    Path(VIDEO_PATH).mkdir(parents=True, exist_ok=True)
    timestamp = str(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    camera_set = env_rel.get_wrapper_attr("camera_set")
    camera_set.record_video(Path(VIDEO_PATH), timestamp)

    # env = RHCWrapper(env, exec_horizon=1)

    controller = ModelInference(env_rel, cfg)
    with env_rel:
        worker = threading.Thread(target=controller.loop, name="model-inference", daemon=True)
        worker.start()
        command_loop(controller)
        worker.join()
        # Finalize (write the moov atom) before the env/camera teardown, which
        # can segfault and skip the normal close() -> stop_video() cleanup,
        # leaving the mp4 files unplayable.
        try:
            camera_set.stop_video()
        except Exception:
            logger.exception("failed to finalize video recording")


if __name__ == "__main__":
    main()
