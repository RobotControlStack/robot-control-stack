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
from vlagents.policies.interface import Obs, SingleAct, SingleObs

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

MAX_REL_MOV_JOINTS = np.deg2rad(20)
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
    jpeg_encoding: bool = False
    on_same_machine: bool = True
    fps: int = 25
    show_camera_windows: bool = False
    n_action_steps: int | None = 25
    max_rel_mov_joints: float = MAX_REL_MOV_JOINTS
    max_rel_mov_cart: tuple[float, float] = MAX_REL_MOV_CART
    # TAM (Torque Adaptation Module): master switch. When enabled, everything
    # else resolves automatically: the default checkpoint is downloaded once
    # into ~/.cache/simadaptor and the ideal-model MJCF is installed with the
    # torque-adaptation-module package.
    tam: bool = True
    # TAM debugging: record the 1 kHz controller/TAM signals into a ring buffer
    # (C++ side) and the 5 Hz history-encoder I/O (Python side), then dump both
    # to CSV on shutdown for offline plotting.
    tam_debug_log: bool = True
    # Resolved at runtime in ModelInference.__init__ (None disables logging).
    tam_log_dir: str | None = None
    # Before the robot moves, drive synthetic history through the encoder until
    # push latency is consistently fast, so JAX has finished compiling every
    # shape the live loop will hit (the runtime only warms the decode step at
    # init; the full push_window path compiles lazily on first real calls).
    tam_warmup: bool = True
    tam_warmup_target_ms: float = 60.0    # a push is "fast" below this
    tam_warmup_streak: int = 8            # consecutive fast pushes required
    tam_warmup_timeout_s: float = 90.0    # give up (and warn) after this
    # Apply franka::limitRate to the final torque. Keep True: FCI trips its own
    # torque-rate reflex otherwise, because PD+residual can sum past ~1 Nm/ms
    # even when neither term alone does.
    rate_limit: bool = True
    # Record the pre-rate-limit intended torque (tau_pd + residual) into TAM's
    # history instead of the post-limiter command, so the rate limiter (kept on
    # for FCI) does not distort the applied-torque history TAM conditions on.
    tam_history_pre_ratelimit: bool = True
    # Per-joint |clip| [Nm] on the TAM residual. The wrist (J7, and to a lesser
    # extent J5/J6) is low-inertia/soft and goes bang-bang at the clip, driving
    # a limit cycle -> choke the wrist authority to test/kill the instability.
    # None keeps the C++ default [10,10,10,10,2,2,2].
    # Sweep J7 up from the stable 0.0: 0.25 -> 0.5 -> 1.0, watching J7 dq / OOD.
    # (0.0/0.5/0.5 was fully stable for 38 s; 2.0 diverged to a velocity fault.)
    tam_residual_clip: tuple | None = (10.0, 10.0, 10.0, 10.0, 0.5, 0.5, 0.5)


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
        pd_mode = 0.0  # pholder for now
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
        self._prev_pd_mode = 0.0
        self.tam_runtime = None
        if cfg.tam_debug_log and cfg.tam_log_dir is None:
            cfg.tam_log_dir = os.path.join(
                os.getcwd(), "tam_logs", time.strftime("%Y%m%d_%H%M%S")
            )
            os.makedirs(cfg.tam_log_dir, exist_ok=True)
            logger.info("TAM debug logs -> %s", cfg.tam_log_dir)
        if cfg.tam:
            self._init_tam()

    def _init_tam(self) -> None:
        """Load the TAM checkpoint: the streaming history encoder runs in this
        process (JAX; a GPU is strongly recommended) and the adaptor MLP is
        exported once into the C++ controller."""
        from simadaptor.deploy.history_runtime import RealTimeHistoryAdaptor

        # from_checkpoint enforces an applied-torque checkpoint: this
        # integration records a single commanded-torque stream.
        self.tam_runtime = RealTimeHistoryAdaptor.from_checkpoint()
        logger.info(
            "TAM ready: adaptor binary %d bytes, applied-torque mode",
            len(self.tam_runtime.adaptor_weight_bytes()),
        )

    def warmup_history_encoder(self) -> None:
        """Compile every JAX path the live encoder will hit, before the robot
        moves. Drives synthetic 1 kHz history through the real ``push_window``
        path (same growing-then-saturated ring the live loop feeds) until pushes
        are consistently under ``tam_warmup_target_ms``, then resets the stream
        so live data starts clean. Blocks until fast or the timeout elapses."""
        if not self._cfg.tam_warmup or self.tam_runtime is None:
            return
        rt = self.tam_runtime
        dof = int(getattr(rt, "_dof", 7))
        ring = 4000          # mirrors the C++ TAM_HISTORY_SIZE
        step = 200           # 1 kHz over one 5 Hz encoder period
        dt = 1e-3
        base_q = np.zeros(dof, dtype=np.float32)
        try:
            base_q[:] = np.asarray(HOME_POSE, dtype=np.float32)[:dof]
        except Exception:
            pass
        phase = np.linspace(0.0, np.pi, dof, dtype=np.float32)

        ts = np.zeros((0,), dtype=np.float64)
        q = np.zeros((0, dof), dtype=np.float32)
        qd = np.zeros((0, dof), dtype=np.float32)
        tau = np.zeros((0, dof), dtype=np.float32)
        grav = np.zeros((0, dof), dtype=np.float32)

        logger.info("TAM warmup: compiling encoder (target < %.0f ms/push)...",
                    self._cfg.tam_warmup_target_ms)
        t_cur = 0.0
        streak = 0
        n_push = 0
        last_ms = float("nan")
        t_start = time.time()
        while time.time() - t_start < self._cfg.tam_warmup_timeout_s:
            tnew = t_cur + dt * np.arange(1, step + 1, dtype=np.float64)
            t_cur = float(tnew[-1])
            w = 2.0 * np.pi * 0.2  # 0.2 Hz gentle motion
            qn = base_q[None, :] + 0.1 * np.sin(w * tnew[:, None] + phase[None, :]).astype(np.float32)
            qdn = (0.1 * w * np.cos(w * tnew[:, None] + phase[None, :])).astype(np.float32)
            taun = (2.0 * np.sin(w * tnew[:, None] + phase[None, :])).astype(np.float32)
            gravn = np.tile(np.array([0, -25, 0, 18, 0, 2, 0][:dof], dtype=np.float32),
                            (step, 1))
            # Append and keep only the most recent `ring` samples, like the C++
            # ring returned by get_tam_history().
            ts = np.concatenate([ts, tnew])[-ring:]
            q = np.concatenate([q, qn])[-ring:]
            qd = np.concatenate([qd, qdn])[-ring:]
            tau = np.concatenate([tau, taun])[-ring:]
            grav = np.concatenate([grav, gravn])[-ring:]

            t0 = time.perf_counter()
            latent = rt.push_window(ts, q, qd, tau, gravity=grav)
            # Force the async JAX result so the timing reflects real compute.
            if latent is not None:
                block = getattr(latent, "block_until_ready", None)
                if block is not None:
                    block()
                else:
                    np.asarray(latent)
            last_ms = (time.perf_counter() - t0) * 1e3
            n_push += 1

            # Only count a push toward the fast streak once the ring is full,
            # a token was actually emitted (latent is not None), and it was fast
            # -- so warmup can't succeed on empty or not-yet-compiled pushes.
            if (
                ts.shape[0] >= ring
                and latent is not None
                and last_ms <= self._cfg.tam_warmup_target_ms
            ):
                streak += 1
            else:
                streak = 0
            if streak >= self._cfg.tam_warmup_streak:
                logger.info("TAM warmup done: %d pushes, last %.1f ms", n_push, last_ms)
                break
        else:
            logger.warning(
                "TAM warmup did not stabilize within %.0fs (last %.1f ms); "
                "expect stalls at the start of the run",
                self._cfg.tam_warmup_timeout_s, last_ms,
            )
        rt.reset()  # discard synthetic history before the live stream

    def run_history_encoder(self):
        """Feed the 1 kHz controller history through the TAM history encoder
        (~5 Hz) and push the resulting latent back into the C++ controller.

        The C++ side applies zero residual until both the MLP weights and the
        first latent have arrived, then ramps the residual in over 1 s."""
        if self.tam_runtime is None:
            logger.info("TAM disabled; history encoder not started")
            return
        robot: Franka = self.env.get_wrapper_attr("envs")["right"].get_wrapper_attr("robot")

        # The weight vector carries the packed adaptor binary, one byte per
        # float64 element (parsed and validated on the C++ side).
        robot.set_tam_mlp_weight(
            np.frombuffer(self.tam_runtime.adaptor_weight_bytes(), dtype=np.uint8).astype(np.float64)
        )

        hist_encoder_framerate = SimpleFrameRate(5)  # 5 Hz latent updates
        latents_sent = 0
        # History-module log (5 Hz): what goes into the encoder (sample span /
        # count / rate) and what comes out (latent norm / dim), so the encoder
        # side can be plotted alongside the 1 kHz C++ debug log.
        hist_log = None
        if self._cfg.tam_log_dir:
            hist_log = open(os.path.join(self._cfg.tam_log_dir, "tam_history_encoder.csv"), "w")
            hist_log.write(
                "wall_t,n_samples,t_first,t_last,span_s,mean_dt_ms,"
                "latent_is_none,latent_norm,latent_dim,push_ms\n"
            )
            hist_log.flush()
        while True:
            try:
                # The runtime handles everything stream-related internally:
                # overlapping polls are deduplicated by timestamp, short holes
                # (e.g. a controller restart during a gain switch) are bridged
                # with masked padding on its dense grid, and a backwards or
                # over-long gap restarts the stream.
                samples = robot.get_tam_history()
                t_push0 = time.perf_counter()
                latent = self.tam_runtime.push_history_samples(samples)
                push_ms = (time.perf_counter() - t_push0) * 1e3
                if latent is not None:
                    latent_arr = np.asarray(latent, dtype=np.float64).reshape((-1,))
                    print(latent_arr.shape)
                    breakpoint()
                    robot.set_tam_latent(latent_arr)
                    latents_sent += 1
                    if latents_sent == 1:
                        logger.info("TAM: first latent sent, residual ramps in on the controller")
                if hist_log is not None:
                    n = len(samples)
                    t_first = float(samples[0].t) if n else 0.0
                    t_last = float(samples[-1].t) if n else 0.0
                    span = t_last - t_first
                    mean_dt_ms = (span / (n - 1) * 1e3) if n > 1 else 0.0
                    if latent is None:
                        l_none, l_norm, l_dim = 1, 0.0, 0
                    else:
                        l_none, l_norm, l_dim = 0, float(np.linalg.norm(latent_arr)), int(latent_arr.size)
                    hist_log.write(
                        f"{time.time():.6f},{n},{t_first:.6f},{t_last:.6f},{span:.6f},"
                        f"{mean_dt_ms:.4f},{l_none},{l_norm:.6f},{l_dim},{push_ms:.4f}\n"
                    )
                    hist_log.flush()
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
        robot = self.env.get_wrapper_attr("robot")["right"]
        if self._cfg.tam_log_dir:
            robot.set_tam_logging(True)
            logger.info("TAM 1 kHz debug logging enabled")
        try:
            while True:
                self.run_episode()
        except KeyboardInterrupt:
            logger.info("interrupted, stopping")
        finally:
            if self._cfg.tam_log_dir:
                try:
                    # Dump with the control thread stopped so there is no
                    # concurrent writer to the debug ring.
                    robot.set_tam_logging(False)
                    robot.stop_control_thread()
                    tag = "tam" if self._cfg.tam else "notam"
                    path = os.path.join(self._cfg.tam_log_dir, f"{tag}_controller_1khz.csv")
                    n = robot.dump_tam_debug_log(path)
                    logger.info("controller log (%s): %d rows -> %s", tag, n, path)
                except Exception:
                    logger.exception("failed to dump TAM controller log")
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
        assert False
        if mode == 1.0:
            # stiff -> lowered to TAM's in-distribution range (base_kp_profile
            # x1.0 for the panda preset) so the TAM residual is not applied
            # out-of-distribution. kd is ~critical damping for these gains.

            # rcfg.kp = np.array([110, 110, 100, 80, 40, 40, 40])
            # rcfg.kd = np.array([21, 21, 20, 18, 12, 13, 13])

            rcfg.kp = np.array([40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0])
            rcfg.kd = np.array([13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0])
            rcfg.torque_limit = np.array([87.0, 87.0, 87.0, 87.0, 87.0, 87.0, 87.0])
            robot.set_config(rcfg)
        else:
            # soft
            assert False
            rcfg.kp = np.array([40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0])
            rcfg.kd = np.array([13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0])
            rcfg.torque_limit = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
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
            # TAM's training convention: the plant sees the ideal model's
            # gravity compensation (see FrankaConfig.tam_ideal_model_path).
            from simadaptor.assets import default_panda_xml

            hw_cfg.robot_cfgs["right"].tam_ideal_model_path = str(default_panda_xml())
        hw_cfg.robot_cfgs["right"].speed_factor = 0.4
        hw_cfg.robot_cfgs["right"].policy_rate = cfg.fps
        hw_cfg.robot_cfgs["right"].allow_high_collision = True
        env_rel = env_creator.create_env(hw_cfg)
        robot = env_rel.get_wrapper_attr("robot")["right"]
        rcfg = robot.get_config()
        # stiff -> lowered to TAM's in-distribution range (base_kp_profile x1.0
        # for the panda preset) so the TAM residual is not applied
        # out-of-distribution. kd is ~critical damping for these gains.
        # rcfg.kp = np.array([110, 110, 100, 80, 40, 40, 40])
        # rcfg.kd = np.array([21, 21, 20, 18, 12, 13, 13])

        rcfg.kp = np.array([40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0])
        rcfg.kd = np.array([13.0, 13.0, 13.0, 13.0, 13.0, 13.0, 13.0])
        rcfg.torque_limit = np.array([87.0, 87.0, 87.0, 87.0, 87.0, 87.0, 87.0])
        rcfg.rate_limit = cfg.rate_limit
        rcfg.tam_history_pre_ratelimit = cfg.tam_history_pre_ratelimit
        if cfg.tam_residual_clip is not None:
            rcfg.tam_residual_clip = np.array(cfg.tam_residual_clip, dtype=np.float64)
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
        # Finish JAX compilation before the robot moves so the first live
        # latents are already fast (no startup stalls -> no stale-latent jumps).
        controller.warmup_history_encoder()

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        input("Press Enter to start the inference loop (Ctrl+C to stop)...")
        if cfg.tam:
            # history_encoder_thread = threading.Thread(
            #     target=controller.run_history_encoder, name="history_encoder", daemon=True
            # )
            # history_encoder_thread.start()
            robot: Franka = controller.env.get_wrapper_attr("envs")["right"].get_wrapper_attr("robot")
            latent = np.load("/home/tobi/Downloads/z_final.npz")["z_final"]
            latent_arr = np.asarray(latent, dtype=np.float64).reshape((-1,))

            golden_input = np.load("/home/tobi/Downloads/golden_input.npz") # with keys: meta_json, t, q, qd, tau_cmd...)
            # breakpoint()
            robot.set_tam_latent(latent_arr)

    with env_rel:
        controller.loop()


if __name__ == "__main__":
    main()
