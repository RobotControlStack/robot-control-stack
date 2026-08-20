import copy
import json
import logging
import threading
from dataclasses import asdict, dataclass, field
from pathlib import Path
from queue import Empty, Queue
from time import sleep
from typing import Any

import cv2
import gymnasium as gym
import numpy as np
import rcs
from rcs._core.common import BaseCameraConfig, RobotPlatform, Pose
from rcs._core.sim import SimConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.utils import SimpleFrameRate

from vlagents.client import RemoteAgent
from vlagents.policies.interface import Act, Obs, SingleAct, SingleObs

logger = logging.getLogger(__name__)


ROBOT2IP = {
    "right": "192.168.1.12",
}

ROBOT_INSTANCE = RobotPlatform.HARDWARE
REALSENSE_CAMERA_DICT = None
ZED_CAMERA_DICT = {
    "gripper_rgb": "14943057",
    "front_rgb": "35115330",
}
HOME_POSE = rcs.HOME_POSITIONS["FR3_DROID"]
ROBOTIQ_SERIAL = {
    "right": "DAANTG8W",
}
INCLUDE_DEPTH = False

CONTROL_MODE = ControlMode.JOINTS
RELATIVETO = RelativeTo.NONE
IP = "localhost"
PORT = 8080

CONFIG_PATH = Path(__file__).with_suffix(".json")

MAX_REL_MOV_JOINTS = np.deg2rad(0.5)
MAX_REL_MOV_CART = (0.5, np.deg2rad(90))

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    level=logging.INFO,
)


@dataclass
class InferenceConfig:
    vlagents_host: str = IP
    vlagents_port: int = PORT
    vlagents_model: str = "maniflow"
    instruction: str = "pick up the green cube"
    robot_keys: list[str] = field(default_factory=lambda: ["right"])
    jpeg_encoding: bool = True
    on_same_machine: bool = False
    fps: int = 30
    record_path: str = "inference_recordings"
    show_camera_windows: bool = False
    n_action_steps: int | None = None
    max_rel_mov_joints: float = MAX_REL_MOV_JOINTS
    max_rel_mov_cart: tuple[float, float] = MAX_REL_MOV_CART


def load_inference_config() -> InferenceConfig:
    if not CONFIG_PATH.exists():
        CONFIG_PATH.write_text(json.dumps(asdict(InferenceConfig()), indent=2) + "\n")
        return InferenceConfig()
    return InferenceConfig(**json.loads(CONFIG_PATH.read_text()))


def normalize_robot_keys(
    cfg: InferenceConfig, available_robots: set[str]
) -> InferenceConfig:
    cfg.robot_keys = [robot for robot in cfg.robot_keys if robot in available_robots]
    if not cfg.robot_keys and "right" in available_robots:
        cfg.robot_keys = ["right"]
    return cfg


def build_vlagent_obs(
    cameras: dict[str, np.ndarray],
    joints: np.ndarray,
    gripper: float,
    instruction: str,
    robot_key: str = "right",
    info: dict | None = None,
) -> Obs:
    single_obs = SingleObs(
        cameras=copy.deepcopy(cameras),
        joints=np.asarray(joints, dtype=np.float32),
        gripper=float(gripper),
        info=dict(info) if info else {},
    )
    return Obs(obs={robot_key: single_obs}, language_instruction=instruction)


def pop_action_step(
    action_buffer: list[dict[str, SingleAct]],
    remote_agent: RemoteAgent,
    obs: Obs,
    n_action_steps: int | None,
) -> dict[str, SingleAct]:
    """Return one action step, refilling ``action_buffer`` from the server as needed."""
    if n_action_steps is None:
        action_chunk = remote_agent.act(obs).acts
        if not action_chunk:
            raise ValueError("Received empty action chunk from policy")
        return action_chunk[0]
    if len(action_buffer) == 0:
        action_chunk = remote_agent.act(obs).acts[:n_action_steps]
        if not action_chunk:
            raise ValueError("Received empty action chunk from policy")
        action_buffer.extend(action_chunk)
    return action_buffer.pop(0)


def unpack_single_act(single_act: SingleAct) -> tuple[np.ndarray, float, float]:
    """Split one agent ``SingleAct`` into ``(joint targets, gripper, pd_mode)``."""
    action = np.asarray(single_act.action, dtype=np.float32)
    gripper = float(
        single_act.gripper
    )  # 1.0 if single_act.gripper is None else float(single_act.gripper)
    if action.shape[0] > 8:
        pd_mode = float(action[8])
    elif action.shape[0] > 7:
        pd_mode = float(action[7])
    else:
        pd_mode = 1.0  # pholder for now
    return action, gripper, pd_mode


class ModelInference:
    def __init__(self, env: gym.Env, cfg: InferenceConfig):
        self.env = env
        self.gripper_state = 1
        self._cfg = cfg
        self._episode_running = False
        self._command_queue: Queue[str] = Queue()
        self._shutdown_requested = threading.Event()
        self.remote_agent = RemoteAgent(
            cfg.vlagents_host,
            cfg.vlagents_port,
            cfg.vlagents_model,
            cfg.on_same_machine,
            cfg.jpeg_encoding,
        )
        self.frame_rate = SimpleFrameRate(self._cfg.fps)
        self._action_buffer = []
        self._camera_windows_enabled = False
        self._prev_pd_mode = 1.0

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

        return (
            start_requested,
            record_requested,
            success_requested,
            stop_requested,
            reload_requested,
        )

    def obs_rcs2agents(self, obs: dict, info: dict | None = None) -> Obs:
        cameras = {}
        for frame in obs["frames"]:
            cameras[frame] = obs["frames"][frame]["rgb"]["data"]

        obs_by_robot = {}
        for robot in self._cfg.robot_keys:
            robot_obs = build_vlagent_obs(
                cameras=cameras,
                joints=obs[robot]["joints"],
                gripper=obs[robot]["gripper"][0],  # (0=closed, 1=open)
                instruction=self._cfg.instruction,
                robot_key=robot,
                info=dict(
                    velo=obs[robot]["robot_state"]["dq_d"],
                    ee_xyz=obs[robot]["tquat_flange"][:3],
                    pd_mode=self._prev_pd_mode,
                ),
            )
            obs_by_robot.update(robot_obs.obs)

        return Obs(obs=obs_by_robot, language_instruction=self._cfg.instruction)

    def act(self, obs_dict: Obs) -> dict[str, SingleAct]:
        """Returns one action step, optionally draining a remote action chunk locally."""
        refill = self._cfg.n_action_steps is not None and len(self._action_buffer) == 0
        action = pop_action_step(
            self._action_buffer, self.remote_agent, obs_dict, self._cfg.n_action_steps
        )
        if refill and RELATIVETO == RelativeTo.CONFIGURED_ORIGIN:
            for robot in self.env.get_wrapper_attr("envs"):
                self.env.get_wrapper_attr("envs")[robot].get_wrapper_attr(
                    "set_origin_to_current"
                )()
        return action

    def action_agents2rcs(self, action: dict[str, SingleAct]) -> dict[str, Any]:
        act = {}
        for robot in self._cfg.robot_keys:
            joints, gripper, pd_mode = unpack_single_act(action[robot])
            act[robot] = {
                # "gripper": np.array([1.0]), #np.asarray([robot_action.gripper], dtype=np.float32),
                "joints": np.asarray(joints, dtype=np.float32),
                "gripper": np.asarray([gripper], dtype=np.float32),
            }
        return act, pd_mode

    def _set_camera_windows_enabled(self, enabled: bool) -> None:
        if enabled == self._camera_windows_enabled:
            return
        if not enabled:
            cv2.destroyAllWindows()
        self._camera_windows_enabled = enabled

    def _show_camera_windows(self, obs: dict[str, Any]) -> None:
        if not self._camera_windows_enabled:
            return
        frames = obs.get("frames", {})
        for frame_name, frame_data in frames.items():
            rgb_frame = frame_data.get("rgb", {}).get("data")
            if rgb_frame is None:
                continue
            cv2.imshow(
                f"camera:{frame_name}", cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            )
        cv2.waitKey(1)

    def loop(self):
        obs, _ = self.env.reset()
        self._set_camera_windows_enabled(self._cfg.show_camera_windows)
        self._show_camera_windows(obs)
        obs_dict = self.obs_rcs2agents(obs)
        logger.info(
            "waiting for input: 'e' to start, 'r' to start and record, 's' for success and reset, 'q' to stop and reset, and 'o' to reload config"
        )

        while not self._shutdown_requested.is_set():
            (
                start_requested,
                record_requested,
                success_requested,
                stop_requested,
                reload_requested,
            ) = self._drain_commands()

            if reload_requested:
                self._cfg = load_inference_config()
                self._set_camera_windows_enabled(self._cfg.show_camera_windows)
                # self._cfg = normalize_robot_keys(self._cfg, set(self.env.get_wrapper_attr("envs")))
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
                    logger.exception(
                        "failed to reconnect after reloading %s", CONFIG_PATH
                    )
                if isinstance(self.env, StorageWrapper):
                    self.env.base_dir = self._cfg.record_path
                    self.env.set_instruction(self._cfg.instruction)
                obs, _ = self.env.reset()
                self._show_camera_windows(obs)
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False

            if success_requested:
                if self._episode_running:
                    logger.info("marking episode successful and resetting environment")
                self.env.get_wrapper_attr("success")()
                obs, _ = self.env.reset()
                self._show_camera_windows(obs)
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False

            if stop_requested:
                if self._episode_running:
                    logger.info("stopping episode and resetting environment")
                obs, _ = self.env.reset()
                self._show_camera_windows(obs)
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
                    logger.info(
                        "starting episode%s",
                        " with recording" if record_requested else "",
                    )
                    self._episode_running = True
                else:
                    sleep(0.05)
                    continue

            # print(obs_dict.cameras.keys())
            # breakpoint()
            action = self.act(copy.deepcopy(obs_dict))
            if any(robot_action.done for robot_action in action.values()):
                logger.info("done issued by agent, resetting environment")
                obs, _ = self.env.reset()
                self._show_camera_windows(obs)
                obs_dict = self.obs_rcs2agents(obs)
                self._action_buffer = []
                self._episode_running = False
                continue
            a, pd_mode = self.action_agents2rcs(action)
            self.pd_mode(pd_mode)
            obs, _, _, _, info = self.env.step(a)

            self._show_camera_windows(obs)
            obs_dict = self.obs_rcs2agents(obs)

            if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
                self.frame_rate()

        self._set_camera_windows_enabled(False)

    def pd_mode(self, mode):
        robot = self.env.get_wrapper_attr("robot")["right"]
        rcfg = robot.get_config()
        if self._prev_pd_mode == mode:
            return
        self._prev_pd_mode = mode
        robot.stop_control_thread()
        if mode == 1.0:
            # stiff
            rcfg.kp = np.array([600, 600, 600, 600, 250, 150, 50])
            rcfg.kd = np.array([50, 50, 50, 50, 30, 25, 15])
            robot.set_config(rcfg)
        else:
            # soft
            rcfg.kp = np.array([40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0])
            rcfg.kd = np.array([13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0])
            robot.set_config(rcfg)


def command_loop(controller: ModelInference) -> None:
    prompt = (
        "Command [e=start, r=record, s=success/reset, q=stop/reset, o=reload, x=exit]: "
    )
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
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_fr3.configs import FrankaDuoEnv, DROIDEnv
        from rcs_fr3.creators import HardwareCameraCreatorConfig

        env_creator = DROIDEnv()
        env_creator.robot_ip = ROBOT2IP["right"]
        env_creator.gripper_serial_number = ROBOTIQ_SERIAL["right"]
        hw_cfg = env_creator.config()
        hw_cfg.robot_cfgs["right"].q_home = HOME_POSE
        hw_cfg.robot_cfgs["right"].tcp_offset = Pose()
        # hw_cfg.wrapper_cfg.home_on_reset = False
        camera_cfgs: dict[str, HardwareCameraCreatorConfig] = {}
        if REALSENSE_CAMERA_DICT is not None:
            camera_cfgs["realsense"] = HardwareCameraCreatorConfig(
                camera_type_id="realsense",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=1280,
                        resolution_height=720,
                        frame_rate=30,
                    )
                    for name, identifier in REALSENSE_CAMERA_DICT.items()
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
                    "enable_depth": INCLUDE_DEPTH,
                    "enable_imu": False,
                    "include_right": False,
                },
            )
        hw_cfg.camera_cfgs = camera_cfgs or None
        hw_cfg.control_mode = CONTROL_MODE
        # async gripper leads to a waiting behaviour until the gripper is fully open/closed, dynamic tasks might not want that
        hw_cfg.gripper_cfgs["right"].async_control = False
        hw_cfg.wrapper_cfg.include_depth = INCLUDE_DEPTH
        hw_cfg.wrapper_cfg.binary_gripper = True
        hw_cfg.max_relative_movement = (
            cfg.max_rel_mov_joints
            if CONTROL_MODE == ControlMode.JOINTS
            else cfg.max_rel_mov_cart
        )
        hw_cfg.relative_to = RELATIVETO
        hw_cfg.robot_to_shared_base_frame = {
            "right": rcs.common.Pose(
                translation=np.array([0, 0, 0]), rpy_vector=np.array([0, 0, 0])
            ),
        }
        hw_cfg.robot_cfgs["right"].ignore_realtime = True
        hw_cfg.robot_cfgs["right"].speed_factor = 0.4
        env_rel = env_creator.create_env(hw_cfg)
        robot = env_rel.get_wrapper_attr("robot")["right"]
        rcfg = robot.get_config()
        # stiff
        rcfg.kp = np.array([600, 600, 600, 600, 250, 150, 50])
        rcfg.kd = np.array([50, 50, 50, 50, 30, 25, 15])
        robot.set_config(rcfg)

    else:
        # FR3

        scene = EmptyWorldFR3()
        sim_cfg_data = scene.config()
        sim_cfg_data.robot_cfgs["right"].q_home = HOME_POSE
        sim_cfg_data.sim_cfg = SimConfig(
            async_control=True,
            realtime=False,
            frequency=cfg.fps,
            max_convergence_steps=500,
        )
        sim_cfg_data.wrapper_cfg.include_depth = INCLUDE_DEPTH
        sim_cfg_data.control_mode = ControlMode.JOINTS
        sim_cfg_data.relative_to = RELATIVETO
        sim_cfg_data.wrapper_cfg.binary_gripper = True
        sim_cfg_data.max_relative_movement = (
            cfg.max_rel_mov_joints
            if CONTROL_MODE == ControlMode.JOINTS
            else cfg.max_rel_mov_cart
        )

        env_rel = scene.create_env(sim_cfg_data)

    return StorageWrapper(
        env_rel,
        cfg.record_path,
        cfg.instruction,
        batch_size=32,
        max_rows_per_group=2,
        max_rows_per_file=10,
        allow_wrapper_instruction=False,
    )


def main() -> None:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    logging.info("Building env")

    cfg = load_inference_config()
    env_rel = get_env(cfg)
    env_rel.reset()
    controller = ModelInference(env_rel, cfg)

    with env_rel:
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            controller.submit_command(
                "e"
            )  # TODO: change this to "r" to record the episode
            controller.loop()
        else:
            worker = threading.Thread(
                target=controller.loop, name="model-inference", daemon=True
            )
            worker.start()
            command_loop(controller)
            worker.join()


if __name__ == "__main__":
    main()
