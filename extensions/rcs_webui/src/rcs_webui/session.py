"""The session runner owns the gym env, the operator (Meta Quest) and the recording.

Everything that touches the env runs in one thread (the runner thread): env creation, stepping, resets and
closing. MuJoCo greenlets and hardware controllers are bound to the thread that created them, and the
recording stays strictly synchronous with the steps. The web server only puts commands into a queue and
reads a status snapshot.

Episode protocol (the same for data collection, human intervention and eval):

- ``A`` starts an episode: the recording of the StorageWrapper is started (and in eval the policy).
- ``B`` ends it as successful: the episode is marked successful, flushed and the env is reset.
- ``Y`` ends it as unsuccessful: the episode is flushed and the env is reset.
- if the env truncates or terminates (e.g. a reflex on the robot), the episode ends as unsuccessful, the
  error is visible in the status and in the recorded data (``truncated`` and ``info.robot_error``).
"""

import copy
import dataclasses
import datetime
import logging
import queue
import re
import threading
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.operator.interface import BaseOperator, TeleopCommands
from rcs.utils import SimpleFrameRate
from rcs_webui import envs, snapshot

logger = logging.getLogger(__name__)

QUEST_CONTROL_MODE = (ControlMode.CARTESIAN_TQuat, RelativeTo.CONFIGURED_ORIGIN)


@dataclass(kw_only=True)
class EnvSpec:
    """What to start: a creator with its default config, a config file or the config of a previous recording,
    each optionally with overrides (dotted paths into the serialized config, see snapshot.apply_overrides)."""

    env: str | None = None
    config_file: Path | None = None
    from_recording: Path | None = None
    overrides: dict[str, Any] = field(default_factory=dict)


@dataclass(kw_only=True)
class SessionSettings:
    data_root: Path
    mode: str = "collect"
    """collect (teleop) or eval (policy with optional human intervention)"""
    instruction: str = ""
    frequency: float = 15.0
    headless: bool = True
    operator: str = "quest"
    """quest or none (episodes are then controlled with the buttons of the web interface)"""
    quest_addr: str = "10.42.0.1"
    quest_switched_left_right: bool = False
    batch_size: int = 32


def _session_dir_name(env_name: str, mode: str) -> str:
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    short = re.sub(r"[^A-Za-z0-9_-]+", "-", env_name.rsplit(":", 1)[-1].rsplit("/", 1)[-1]).strip("-")
    return f"{stamp}_{short}_{mode}"


class SessionRunner(threading.Thread):
    """Runs the env loop in its own thread, see the module docstring."""

    def __init__(self, settings: SessionSettings, env_spec: EnvSpec, operator: BaseOperator | None = None):
        super().__init__(daemon=True, name="rcs-webui-session")
        self.settings = settings
        self.env_spec = env_spec
        self._operator = operator
        self._commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._lock = threading.Lock()
        self._exit = threading.Event()
        self.env: gym.Env | None = None
        self.storage: StorageWrapper | None = None
        self.session_dir: Path | None = None
        self._cfg: Any = None
        self._robot_names: list[str] = []
        self._control_mode: tuple[ControlMode, RelativeTo] | None = None
        self._last_obs: dict[str, Any] = {}
        self._episode_active = False
        self._status: dict[str, Any] = {
            "state": "starting",
            "mode": settings.mode,
            "env": None,
            "session_dir": None,
            "episode_active": False,
            "episode_steps": 0,
            "episode_uuid": None,
            "episodes": {"successful": 0, "unsuccessful": 0, "truncated": 0},
            "last_event": None,
            "error": None,
            "step_hz": 0.0,
            "instruction": settings.instruction,
        }

    # ---------------------------------------------------------------- public, thread safe
    def status(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._status)

    def submit(self, command: str, payload: Any = None):
        """Commands: button (A, B or Y), restart (EnvSpec or None for the same spec), instruction (str), stop."""
        self._commands.put((command, payload))

    def stop(self, timeout: float | None = 30.0):
        self.submit("stop")
        self._exit.set()
        self.join(timeout)

    def latest_frame(self, camera: str) -> np.ndarray | None:
        frames = self._last_obs.get("frames") if isinstance(self._last_obs, dict) else None
        if not frames or camera not in frames:
            return None
        return frames[camera].get("rgb", {}).get("data")

    def cameras(self) -> list[str]:
        frames = self._last_obs.get("frames") if isinstance(self._last_obs, dict) else None
        return list(frames) if frames else []

    # ---------------------------------------------------------------- internals
    def _set_status(self, **kwargs):
        with self._lock:
            self._status.update(kwargs)

    def _event(self, message: str, level: int = logging.INFO):
        logger.log(level, message)
        self._set_status(last_event=f"{datetime.datetime.now().strftime('%H:%M:%S')} {message}")

    def _resolve_config(self, spec: EnvSpec) -> tuple[Any, str, dict[str, Any]]:
        """Returns the config, the creator name and how it was obtained."""
        source: dict[str, Any] = {"overrides": spec.overrides}
        if spec.from_recording is not None:
            cfg, meta = snapshot.load_session_config(Path(spec.from_recording))
            name = meta.get("creator")
            if name is None:
                msg = f"{spec.from_recording} has no creator in its session.json"
                raise ValueError(msg)
            source["from_recording"] = str(spec.from_recording)
        elif spec.config_file is not None:
            cfg = snapshot.config_from_json(Path(spec.config_file).read_text())
            if spec.env is None:
                msg = "a config file needs the env (creator) it belongs to"
                raise ValueError(msg)
            name = envs.creator_name(envs.load_creator(spec.env))
            source["config_file"] = str(spec.config_file)
        else:
            if spec.env is None:
                msg = "no env given"
                raise ValueError(msg)
            creator = envs.load_creator(spec.env)
            name = envs.creator_name(creator)
            cfg = envs.prepare_config(
                creator.config(),
                frequency=self.settings.frequency,
                control_mode=QUEST_CONTROL_MODE if self.settings.mode == "collect" else None,
                headless=self.settings.headless,
            )
            source["default_config_of"] = spec.env
        if spec.overrides:
            cfg = snapshot.from_jsonable(snapshot.apply_overrides(snapshot.to_jsonable(cfg), spec.overrides))
        return cfg, name, source

    def _close_env(self):
        if self.storage is not None:
            # flushes a running episode (unsuccessful) and waits for the writer
            try:
                self.storage.close()
            except Exception:
                logger.exception("closing the env failed")
        self.env = None
        self.storage = None
        self._episode_active = False

    def _start_env(self, spec: EnvSpec):
        self._close_env()
        self._set_status(state="starting", error=None, episode_active=False, episode_steps=0)
        cfg, name, source = self._resolve_config(spec)
        creator = envs.load_creator(name)
        session_dir = Path(self.settings.data_root) / _session_dir_name(spec.env or name, self.settings.mode)
        snapshot.write_snapshot(
            session_dir,
            cfg,
            {
                "creator": name,
                "mode": self.settings.mode,
                "instruction": self.settings.instruction,
                "frequency": self.settings.frequency,
                "operator": self.settings.operator,
                "source": source,
                "data_dir": snapshot.DATA_DIR,
            },
        )
        env = creator.create_env(cfg)
        self.storage = StorageWrapper(
            env,
            str(session_dir / snapshot.DATA_DIR),
            self.settings.instruction,
            batch_size=self.settings.batch_size,
            always_record=False,
        )
        self.env = self.storage
        self.session_dir = session_dir
        self._cfg = cfg
        self._control_mode = (cfg.control_mode, cfg.relative_to) if hasattr(cfg, "relative_to") else None
        self.env_spec = spec
        robot_envs = self.env.get_wrapper_attr("envs")
        self._robot_names = list(robot_envs) if isinstance(robot_envs, dict) else []
        if self._operator is not None:
            sim = self.env.get_wrapper_attr("sim") if envs.is_sim_config(cfg) else None
            self._operator.sim = sim
        self._reset_env()
        self._set_status(state="ready", env=name, session_dir=str(session_dir))
        self._event(f"env {name} started, recording into {session_dir}")

    def _reset_env(self):
        assert self.env is not None
        self._last_obs, _ = self.env.reset()
        if self._operator is not None:
            self._operator.set_camera(self._last_obs)
            self._operator.reset_operator_state()
        self._episode_active = False
        self._set_status(episode_active=False, episode_steps=0, episode_uuid=None)

    def _hold_action(self) -> dict[str, Any]:
        """Action that keeps every robot where it is."""
        actions: dict[str, Any] = {}
        mode, relative_to = self._control_mode or QUEST_CONTROL_MODE
        for robot in self._robot_names:
            obs = self._last_obs.get(robot, {})
            act: dict[str, Any] = {}
            if relative_to != RelativeTo.NONE:
                if mode == ControlMode.JOINTS:
                    act["joints"] = np.zeros_like(obs.get("joints", np.zeros(7)))
                elif mode == ControlMode.CARTESIAN_TRPY:
                    act["xyzrpy"] = np.zeros(6)
                else:
                    act["tquat"] = np.array([0, 0, 0, 0, 0, 0, 1.0])
            elif mode == ControlMode.JOINTS:
                act["joints"] = np.asarray(obs["joints"]).copy()
            elif mode == ControlMode.CARTESIAN_TRPY:
                act["xyzrpy"] = np.asarray(obs["xyzrpy"]).copy()
            else:
                act["tquat"] = np.asarray(obs["tquat"]).copy()
            if "gripper" in obs:
                act["gripper"] = np.asarray(obs["gripper"], dtype=np.float32).reshape(-1)
            actions[robot] = act
        return actions

    def _operator_action(self) -> dict[str, Any]:
        actions = self._hold_action()
        if self._operator is None:
            return actions
        for robot, act in self._operator.consume_action().items():
            if robot in actions:
                actions[robot] = act
        return actions

    def _end_episode(self, success: bool, reason: str):
        assert self.storage is not None
        if self._episode_active:
            if success:
                self.storage.success()
            with self._lock:
                key = "successful" if success else ("truncated" if reason == "truncated" else "unsuccessful")
                self._status["episodes"][key] += 1
            self._event(f"episode ended: {reason}")
        self._reset_env()

    def _start_episode(self):
        assert self.storage is not None
        if self._episode_active:
            return
        self.storage.set_instruction(self.settings.instruction)
        self.storage.start_record()
        self._episode_active = True
        self._set_status(episode_active=True, episode_steps=0, episode_uuid=self.storage.uuid.hex)
        self._event("episode started")

    def _handle_buttons(self, cmds: TeleopCommands) -> bool:
        """Returns True if the env was reset."""
        if cmds.record:
            self._start_episode()
        if cmds.success:
            self._end_episode(True, "successful (B)")
            return True
        if cmds.failure:
            self._end_episode(False, "unsuccessful (Y)")
            return True
        return False

    def _process_commands(self) -> TeleopCommands:
        web = TeleopCommands()
        while True:
            try:
                command, payload = self._commands.get_nowait()
            except queue.Empty:
                break
            if command == "stop":
                self._exit.set()
            elif command == "button":
                web.record |= payload == "A"
                web.success |= payload == "B"
                web.failure |= payload == "Y"
            elif command == "restart":
                try:
                    self._start_env(payload or self.env_spec)
                except Exception as e:
                    self._fail(e)
            elif command == "instruction":
                self.settings.instruction = str(payload)
                self._set_status(instruction=self.settings.instruction)
        return web

    def _fail(self, e: BaseException):
        logger.exception("session error")
        self._set_status(state="error", error="".join(traceback.format_exception_only(type(e), e)).strip())
        self._close_env()

    def _tick(self, web: TeleopCommands):
        assert self.env is not None
        cmds = self._operator.consume_commands() if self._operator is not None else TeleopCommands()
        cmds.record |= web.record
        cmds.success |= web.success
        cmds.failure |= web.failure
        if self._handle_buttons(cmds):
            return
        for controller, reset in cmds.reset_origin_to_current.items():
            robot_envs = self.env.get_wrapper_attr("envs")
            if reset and controller in robot_envs:
                robot_envs[controller].get_wrapper_attr("set_origin_to_current")()
        self._last_obs, _, terminated, truncated, info = self.env.step(self._operator_action())
        if self._operator is not None:
            self._operator.set_camera(self._last_obs)
        if self._episode_active:
            with self._lock:
                self._status["episode_steps"] += 1
        if terminated or truncated:
            errors = {k: v.get("robot_error") for k, v in info.items() if isinstance(v, dict) and v.get("robot_error")}
            self._set_status(
                error=f"episode ended by the env: {errors or ('terminated' if terminated else 'truncated')}"
            )
            self._end_episode(False, "truncated")

    def run(self):
        try:
            self._start_env(self.env_spec)
        except Exception as e:
            self._fail(e)
        rate = SimpleFrameRate(None, "webui")
        last = time.perf_counter()
        while not self._exit.is_set():
            web = self._process_commands()
            if self._exit.is_set():
                break
            if self.env is None:
                time.sleep(0.05)
                continue
            try:
                self._tick(web)
            except Exception as e:
                self._fail(e)
                continue
            now = time.perf_counter()
            self._set_status(
                step_hz=round(1.0 / max(now - last, 1e-6), 1), state="recording" if self._episode_active else "ready"
            )
            last = now
            rate()
        self._close_env()
        self._set_status(state="stopped")


def with_overrides(spec: EnvSpec, **changes: Any) -> EnvSpec:
    return dataclasses.replace(spec, **changes)
