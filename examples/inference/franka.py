import copy
import datetime
import json
import logging
from dataclasses import asdict, dataclass, field
from pathlib import Path
from time import sleep
from typing import Any
from PIL import Image
import threading

from rcs.utils import SimpleFrameRate

import gymnasium as gym
import numpy as np
from rcs._core.common import BaseCameraConfig, RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3Duo
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.envs.tasks import PickTaskConfig

import rcs
from rcs_duobench.tasks.bin_sort import BinSortEnvConfig
from vlagents.client import RemoteAgent
from vlagents.policies import Act, Obs

from pynput import keyboard

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

# set camera dict to none disable cameras
CAMERA_DICT = {
    "left_wrist": "230422272017",
    "right_wrist": "230422271040",
    # "side": "243522070385",
    # "bird_eye": "243522070364",
}
# CAMERA_DICT = None
ZED_CAMERA_DICT = {
    "head": "19928076",
}
INCLUDE_DEPTH = False

ROBOTIQ_SERIAL = {
    "left": "DAAQMPDC",
    "right": "DAAQMJHX",
}

# DIGIT_DICT = {
#     "digit_right_left": "D21182",
#     "digit_right_right": "D21193"
# }
DIGIT_DICT = None


INSTRUCTION = "pick up the black cube with the right arm and place it into the black bowl; pick up the white cube with the left arm and place it into the white bowl"
FPS = 30
CONTROL_MODE = ControlMode.JOINTS
RELATIVETO = RelativeTo.NONE
RECORD_PATH = "inference_recordings"
MODEL = "lerobot"
IP = "localhost"
PORT = 20000
CONFIG_PATH = Path(__file__).with_suffix(".json")

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    level=logging.INFO,
)


robot2world = {
    "right": rcs.common.Pose(
        translation=np.array([0, 0, 0]), rpy_vector=np.array([0.89360858, -0.17453293, 0.46425758])
    ),
    "left": rcs.common.Pose(
        translation=np.array([0, 0, 0]), rpy_vector=np.array([-0.89360858, -0.17453293, -0.46425758])
    ),
}


@dataclass
class InferenceConfig:
    vlagents_host: str = IP
    vlagents_port: int = PORT
    vlagents_model: str = MODEL
    instruction: str = INSTRUCTION
    robot_keys: list[str] = field(default_factory=lambda: ["left", "right"])
    jpeg_encoding: bool = True
    on_same_machine: bool = False
    fps: int = FPS
    record_path: str = RECORD_PATH
    n_action_steps: int | None = None


def load_inference_config() -> InferenceConfig:
    if not CONFIG_PATH.exists():
        CONFIG_PATH.write_text(json.dumps(asdict(InferenceConfig()), indent=2) + "\n")
        return InferenceConfig()
    return InferenceConfig(**json.loads(CONFIG_PATH.read_text()))



class ModelInference:
    def __init__(self, env: gym.Env, cfg: InferenceConfig):
        self.env = env
        self.gripper_state = 1
        self._cfg = cfg
        self._episode_running = False
        self._start_requested = False
        self._record_requested = False
        self._stop_requested = False
        self._reload_requested = False
        self._cmd_lock = threading.Lock()
        self.remote_agent = RemoteAgent(
            cfg.vlagents_host, cfg.vlagents_port, cfg.vlagents_model, cfg.on_same_machine, cfg.jpeg_encoding
        )
        self.frame_rate = SimpleFrameRate(self._cfg.fps)
        self._listener = keyboard.Listener(on_press=self._on_press)
        self._listener.start()
        self._action_buffer = []

    def _on_press(self, key):
        try:
            if not hasattr(key, "char"):
                return
            if key.char == "e":
                with self._cmd_lock:
                    self._start_requested = True
            elif key.char == "r":
                with self._cmd_lock:
                    self._record_requested = True
            elif key.char == "q":
                with self._cmd_lock:
                    self._stop_requested = True
            elif key.char == "o":
                with self._cmd_lock:
                    self._reload_requested = True
        except AttributeError:
            pass

    def obs_rcs2agents(self, obs: dict, info: dict | None = None) -> Obs:
        cameras = {}
        for frame in obs["frames"]:
            cameras[frame] = obs["frames"][frame]["rgb"]["data"]
            cameras[frame] = np.array(Image.fromarray(cameras[frame]).resize((224, 224), Image.Resampling.LANCZOS))

        state = []
        for robot in self._cfg.robot_keys:
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
        act = self._action_buffer.pop(0)
        return Act(action=act, done=done)

    def action_agents2rcs(self, action: Act) -> dict[str, Any]:
        act = {}
        for idx, robot in enumerate(self._cfg.robot_keys):
            # TODO: this is currently hard coded for franka joints
            act[robot] = {}
            act[robot]["joints"] = action.action[idx * 8 : idx * 8 + 7]
            act[robot]["gripper"] = action.action[idx * 8 + 7 : idx * 8 + 8]
        return act

    def loop(self):
        obs, _ = self.env.reset()
        obs_dict = self.obs_rcs2agents(obs)
        logger.info("waiting for 'e' to start, 'r' to start and record, 'q' to stop and reset, and 'o' to reload config")

        while True:
            with self._cmd_lock:
                start_requested = self._start_requested
                record_requested = self._record_requested
                stop_requested = self._stop_requested
                reload_requested = self._reload_requested
                self._start_requested = False
                self._record_requested = False
                self._stop_requested = False
                self._reload_requested = False

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
                obs, _ = self.env.reset()
                obs_dict = self.obs_rcs2agents(obs)
                self._episode_running = False

            if stop_requested:
                if self._episode_running:
                    logger.info("stopping episode and resetting environment")
                obs, _ = self.env.reset()
                obs_dict = self.obs_rcs2agents(obs)
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
                obs, _ = self.env.reset()
                obs_dict = self.obs_rcs2agents(obs)
                self._episode_running = False
                continue
            a = self.action_agents2rcs(action)
            obs, _, _, _, info = self.env.step(a)
            # print(obs["left"]["joints"], obs["left"]["gripper"], obs["right"]["joints"], obs["right"]["gripper"])

            obs_dict = self.obs_rcs2agents(obs)

            if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
                self.frame_rate()


def get_env(cfg: InferenceConfig) -> gym.Env:
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
        hw_cfg.control_mode = CONTROL_MODE
        hw_cfg.wrapper_cfg.include_depth = INCLUDE_DEPTH
        hw_cfg.max_relative_movement = 0.5 if CONTROL_MODE == ControlMode.JOINTS else (0.5, np.deg2rad(90))
        hw_cfg.relative_to = RELATIVETO
        hw_cfg.robot_to_shared_base_frame = robot2world
        hw_cfg.robot_cfgs["left"].ignore_realtime = True
        hw_cfg.robot_cfgs["right"].ignore_realtime = True
        hw_cfg.robot_cfgs["left"].speed_factor = 0.1
        hw_cfg.robot_cfgs["right"].speed_factor = 0.1
        hw_cfg.gripper_cfgs["left"].serial_number = ROBOTIQ_SERIAL["left"]
        hw_cfg.gripper_cfgs["right"].serial_number = ROBOTIQ_SERIAL["right"]
        env_rel = env_creator.create_env(hw_cfg)
    else:
        # FR3

        scene = BinSortEnvConfig()
        sim_cfg_data = scene.config()
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True, realtime=False, frequency=cfg.fps, max_convergence_steps=500
        )
        sim_cfg_data.relative_to = RelativeTo.CONFIGURED_ORIGIN
        sim_cfg_data.wrapper_cfg.include_depth = INCLUDE_DEPTH
        sim_cfg_data.control_mode = ControlMode.JOINTS
        sim_cfg_data.relative_to = RELATIVETO
        sim_cfg_data.wrapper_cfg.binary_gripper = True


        # if sim_cfg_data.root_frame_objects is None:
        #     sim_cfg_data.root_frame_objects = {}
        # sim_cfg_data.task_cfg = PickTaskConfig(robot_name="right")

        env_rel = scene.create_env(sim_cfg_data)

    return StorageWrapper(
        env_rel,
        cfg.record_path,
        cfg.instruction,
        batch_size=32,
        max_rows_per_group=2,
        max_rows_per_file=10,
    )


def main():
    cfg = load_inference_config()
    env_rel = get_env(cfg)
    env_rel.reset()

    # Path(VIDEO_PATH).mkdir(parents=True, exist_ok=True)
    # timestamp = str(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    # camera_set = env_rel.get_wrapper_attr("camera_set")
    # camera_set.record_video(Path(VIDEO_PATH), timestamp)

    # env = RHCWrapper(env, exec_horizon=1)

    controller = ModelInference(env_rel, cfg)
    with env_rel:
        controller.loop()


if __name__ == "__main__":
    main()
