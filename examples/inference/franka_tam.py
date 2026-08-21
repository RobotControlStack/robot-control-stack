import copy
import logging
import os
import threading
import time

# The TAM history encoder (JAX) shares the GPU with everything else on this
# machine; never let JAX preallocate ~75% of device memory. Must be set
# before JAX initializes its backend (first use inside _init_tam).
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
from dataclasses import dataclass, field
from typing import Any

import cv2
import gymnasium as gym
import numpy as np
from rcs._core.common import BaseCameraConfig, Pose, RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3
from rcs.utils import SimpleFrameRate
from rcs_fr3._core.hw import Franka
from vlagents.client import RemoteAgent
from vlagents.policies.interface import Act, Obs, SingleAct, SingleObs

import rcs

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
    show_camera_windows: bool = False
    n_action_steps: int | None = None
    max_rel_mov_joints: float = MAX_REL_MOV_JOINTS
    max_rel_mov_cart: tuple[float, float] = MAX_REL_MOV_CART
    # TAM (Torque Adaptation Module): master switch. When enabled, the
    # checkpoint and ideal-model MJCF resolve automatically: the default
    # checkpoint is downloaded once into ~/.cache/simadaptor and the MJCF is
    # installed with the torque-adaptation-module package.
    tam: bool = False
    # Checkpoint directory override (save_dict.pkl or a checkpoint_<step>
    # dir); None fetches simadaptor.assets.DEFAULT_CHECKPOINT.
    tam_ckpt: str | None = None
    # Ideal-model MJCF override; None uses the packaged panda_pandagripper.xml.
    tam_xml: str | None = None
    tam_attention_history_s: float = 4.0
    tam_latent_rate_hz: float = 5.0


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
    gripper = float(single_act.gripper)  # 1.0 if single_act.gripper is None else float(single_act.gripper)
    if action.shape[0] > 8:
        pd_mode = float(action[8])
    else:
        pd_mode = 1.0  # pholder for now
    return action, gripper, pd_mode


class ModelInference:
    def __init__(self, env: gym.Env, cfg: InferenceConfig):
        self.env = env
        self._cfg = cfg
        self.remote_agent = RemoteAgent(
            cfg.vlagents_host,
            cfg.vlagents_port,
            cfg.vlagents_model,
            cfg.on_same_machine,
            cfg.jpeg_encoding,
        )
        self.frame_rate = SimpleFrameRate(self._cfg.fps)
        self._action_buffer = []
        self._prev_pd_mode = 1.0
        self.tam_runtime = None
        self.tam_weight_bytes: bytes | None = None
        if cfg.tam:
            self._init_tam()

    def _init_tam(self) -> None:
        """Load the TAM checkpoint: the streaming history encoder runs in this
        process (JAX; a GPU is strongly recommended) and the adaptor MLP is
        exported once into the C++ controller."""
        from simadaptor.deploy.history_runtime import RealTimeHistoryAdaptor

        # from_checkpoint enforces an applied-torque checkpoint: this
        # integration records a single commanded-torque stream.
        self.tam_runtime = RealTimeHistoryAdaptor.from_checkpoint(
            self._cfg.tam_ckpt,
            xml_path=self._cfg.tam_xml,
            attention_history_s=float(self._cfg.tam_attention_history_s),
        )
        self.tam_weight_bytes = self.tam_runtime.adaptor_weight_bytes()
        logger.info("TAM ready: adaptor binary %d bytes, applied-torque mode", len(self.tam_weight_bytes))

    def run_history_encoder(self):
        """Feed the 1 kHz controller history through the TAM history encoder
        (~5 Hz) and push the resulting latent back into the C++ controller.

        The C++ side applies zero residual until both the MLP weights and the
        first latent have arrived, then ramps the residual in over 1 s."""
        if self.tam_runtime is None or self.tam_weight_bytes is None:
            logger.info("TAM disabled; history encoder not started")
            return
        robot: Franka = self.env.get_wrapper_attr("envs")["right"].get_wrapper_attr("robot")()

        # The weight vector carries the packed adaptor binary, one byte per
        # float64 element (parsed and validated on the C++ side).
        robot.set_tam_mlp_weight(np.frombuffer(self.tam_weight_bytes, dtype=np.uint8).astype(np.float64))

        hist_encoder_framerate = SimpleFrameRate(int(self._cfg.tam_latent_rate_hz))
        latents_sent = 0
        while True:
            try:
                hist = robot.get_tam_history()
                if len(hist) == 0:
                    hist_encoder_framerate()
                    continue
                t = np.asarray([s.t for s in hist], dtype=np.float64)
                q = np.asarray([s.q for s in hist], dtype=np.float32)
                dq = np.asarray([s.dq for s in hist], dtype=np.float32)
                tau_cmd = np.asarray([s.tau_cmd for s in hist], dtype=np.float32)
                gravity = np.asarray([s.gravity for s in hist], dtype=np.float32)

                # tau_cmd is the gravity-free commanded torque; the runtime
                # combines it with the logged gravity into the ideal-model
                # (gravity-included) torque the encoder was trained on. The
                # runtime also handles everything stream-related internally:
                # overlapping polls are deduplicated by timestamp, short holes
                # (e.g. a controller restart during a gain switch) are bridged
                # with masked padding on its dense grid, and a backwards or
                # over-long gap restarts the stream.
                latent = self.tam_runtime.push_window(t, q, dq, tau_cmd, gravity=gravity)
                if latent is not None:
                    robot.set_tam_latent(np.asarray(latent, dtype=np.float64).reshape((-1,)))
                    latents_sent += 1
                    if latents_sent == 1:
                        logger.info("TAM: first latent sent, residual ramps in on the controller")
            except Exception:
                logger.exception("TAM history encoder step failed; retrying")
                time.sleep(0.5)
            hist_encoder_framerate()

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
        action = pop_action_step(self._action_buffer, self.remote_agent, obs_dict, self._cfg.n_action_steps)
        if refill and RELATIVETO == RelativeTo.CONFIGURED_ORIGIN:
            for robot in self.env.get_wrapper_attr("envs"):
                self.env.get_wrapper_attr("envs")[robot].get_wrapper_attr("set_origin_to_current")()
        return action

    def action_agents2rcs(self, action: dict[str, SingleAct]) -> dict[str, Any]:
        act = {}
        for robot in self._cfg.robot_keys:
            joints, gripper, pd_mode = unpack_single_act(action[robot])
            act[robot] = {
                "joints": np.asarray(joints, dtype=np.float32),
                "gripper": np.asarray([gripper], dtype=np.float32),
            }
        return act, pd_mode

    def _show_camera_windows(self, obs: dict[str, Any]) -> None:
        if not self._cfg.show_camera_windows:
            return
        frames = obs.get("frames", {})
        for frame_name, frame_data in frames.items():
            rgb_frame = frame_data.get("rgb", {}).get("data")
            if rgb_frame is None:
                continue
            cv2.imshow(f"camera:{frame_name}", cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

    def run_episode(self) -> None:
        """Run a single episode until the agent issues ``done``."""
        self.remote_agent.ensure_connected()
        self._action_buffer = []
        obs, _ = self.env.reset()
        logger.info("starting episode")
        self._show_camera_windows(obs)
        obs_dict = self.obs_rcs2agents(obs)

        while True:
            action = self.act(copy.deepcopy(obs_dict))
            if any(robot_action.done for robot_action in action.values()):
                logger.info("done issued by agent, ending episode")
                return
            a, pd_mode = self.action_agents2rcs(action)
            self.pd_mode(pd_mode)
            obs, _, _, _, info = self.env.step(a)

            self._show_camera_windows(obs)
            obs_dict = self.obs_rcs2agents(obs)

            if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
                self.frame_rate()

    def loop(self) -> None:
        """Run episodes back to back until interrupted."""
        try:
            while True:
                self.run_episode()
        except KeyboardInterrupt:
            logger.info("interrupted, stopping")
        finally:
            self.env.reset()
            if self._cfg.show_camera_windows:
                cv2.destroyAllWindows()

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


def get_env(cfg: InferenceConfig) -> gym.Env:
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_fr3.configs import DROIDEnv, FrankaDuoEnv
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
            cfg.max_rel_mov_joints if CONTROL_MODE == ControlMode.JOINTS else cfg.max_rel_mov_cart
        )
        hw_cfg.relative_to = RELATIVETO
        hw_cfg.robot_to_shared_base_frame = {
            "right": rcs.common.Pose(translation=np.array([0, 0, 0]), rpy_vector=np.array([0, 0, 0])),
        }
        hw_cfg.robot_cfgs["right"].ignore_realtime = True
        hw_cfg.robot_cfgs["right"].tam_enabled = cfg.tam
        if cfg.tam:
            # SCHED_FIFO for the 1 kHz thread (best-effort; needs an rtprio
            # rlimit): the TAM residual leaves no deadline slack on a stock
            # kernel otherwise.
            hw_cfg.robot_cfgs["right"].rt_priority = 80
        hw_cfg.robot_cfgs["right"].speed_factor = 0.4
        hw_cfg.robot_cfgs["right"].policy_rate = cfg.fps
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
            cfg.max_rel_mov_joints if CONTROL_MODE == ControlMode.JOINTS else cfg.max_rel_mov_cart
        )

        env_rel = scene.create_env(sim_cfg_data)

    return env_rel


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    logging.info("Building env")

    cfg = InferenceConfig()
    env_rel = get_env(cfg)
    controller = ModelInference(env_rel, cfg)

    if cfg.tam:
        history_encoder_thread = threading.Thread(
            target=controller.run_history_encoder, name="history_encoder", daemon=True
        )
        history_encoder_thread.start()

    with env_rel:
        controller.loop()


if __name__ == "__main__":
    main()
