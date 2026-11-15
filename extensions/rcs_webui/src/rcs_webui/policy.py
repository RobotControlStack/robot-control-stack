"""Client interface to policy servers for evaluation, the chunk executor with optional real-time chunking (RTC)
and the built-in clients (VLAgents, mock).

A policy server is plugged in by implementing :class:`PolicyClient` and selecting it by name: a built-in name
(``vlagents``, ``mock``), an installed entry point of the group ``rcs_webui.policy_clients`` or an import path
``my_package.my_module:MyClient``. RCS never imports a project specific client itself.

Action chunks are kept in two forms: ``actions`` are RCS env actions (one dict per step, keyed by robot name,
ready for ``env.step``) and ``raw`` is the chunk in the server's own action space (``H x A`` array). Real-time
chunking sends parts of ``raw`` back to the server, so the server never has to understand RCS actions.

Real-time chunking (off by default, see :class:`ChunkExecutor`): the server is queried asynchronously
``delay`` steps before the switch-over step of the current chunk. The query carries the committed part of the
current chunk (the ``delay`` actions that are executed while the server computes, ``prefix``) and the rest of the
current chunk (the switch-over part, ``guidance``) which the server may inpaint. The returned chunk is aligned
to the step at which the query was sent, its first ``delay`` actions correspond to the prefix. The robot loop
keeps executing, checks for the answer and switches to the newest chunk at the switch-over step (``index =
steps since the query``); if the answer is late it switches as soon as it arrives at the aligned index.
"""

import abc
import importlib
import itertools
import logging
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from importlib.metadata import entry_points
from typing import Any, ClassVar

import numpy as np

logger = logging.getLogger(__name__)

ENTRY_POINT_GROUP = "rcs_webui.policy_clients"


@dataclass(frozen=True)
class PolicyCapabilities:
    """What a client (and its server) supports."""

    realtime_chunking: bool = False
    """The server accepts ``prefix`` / ``guidance`` / ``delay`` and returns a chunk aligned to the query step."""
    reconnect: bool = True
    """Host and port can be changed at runtime."""


@dataclass(kw_only=True)
class ChunkRequest:
    """One query to the policy server."""

    request_id: int
    observation: dict[str, Any]
    """The RCS observation of the multi robot env (keyed by robot name, plus ``frames`` with the cameras)."""
    instruction: str
    episode_step: int
    """Episode step whose observation is sent, the returned chunk starts at this step."""
    prefix: np.ndarray | None = None
    """RTC: ``delay x A`` raw actions of the current chunk that are executed while the server computes. The
    server must keep them (hard constraint, e.g. clean prefix tokens) and return them as the first ``delay``
    actions."""
    guidance: np.ndarray | None = None
    """RTC: ``k x A`` raw actions of the current chunk after the prefix (the switch-over part) that the new chunk
    should blend into (soft constraint / inpainting). May be empty."""
    delay: int = 0
    """RTC: number of steps executed between sending the query and the switch-over step (= ``len(prefix)``)."""
    prefix_chunk_id: int = -1
    """RTC: request id of the chunk the prefix and guidance were cut from (-1 without RTC). Lets a stateful server
    use its own (e.g. normalized) copy of that chunk instead of the raw actions."""
    prefix_start: int = 0
    """RTC: index of the first prefix action in that chunk."""


@dataclass(kw_only=True)
class ActionChunk:
    """Answer of the policy server."""

    actions: list[dict[str, Any]]
    """RCS env actions, one per step, aligned to ``ChunkRequest.episode_step``."""
    raw: np.ndarray | None = None
    """The same chunk in the server's action space (``H x A``), needed for RTC."""
    metadata: dict[str, Any] = field(default_factory=dict)
    """Anything the server reports (e.g. inference time), logged but not interpreted."""
    done: bool = False
    """The policy considers the episode finished."""


class PolicyClient(abc.ABC):
    """Interface of a policy server client. Implementations must be safe to call from one worker thread."""

    capabilities: ClassVar[PolicyCapabilities] = PolicyCapabilities()

    def __init__(self, host: str, port: int, **options: Any):
        self.host = host
        self.port = port
        self.options = options

    @abc.abstractmethod
    def connect(self) -> None:
        """Connects to the server (or checks that it is reachable). May be called again after a failure."""

    def reset(self, instruction: str) -> None:  # noqa: B027
        """Called at the start of every episode."""

    @abc.abstractmethod
    def infer(self, request: ChunkRequest) -> ActionChunk:
        """Returns the action chunk for the request (blocking)."""

    def close(self) -> None:  # noqa: B027
        """Releases the connection."""

    def describe(self) -> dict[str, Any]:
        return {
            "client": f"{type(self).__module__}:{type(self).__qualname__}",
            "host": self.host,
            "port": self.port,
            "realtime_chunking": self.capabilities.realtime_chunking,
        }


def load_policy_client_class(name: str) -> type[PolicyClient]:
    """Resolves ``vlagents``, ``mock``, an entry point name of ``rcs_webui.policy_clients`` or ``module:Class``."""
    if name in BUILTIN_CLIENTS:
        name = BUILTIN_CLIENTS[name]
    elif ":" not in name:
        matches = [ep for ep in entry_points(group=ENTRY_POINT_GROUP) if ep.name == name]
        if not matches:
            msg = f"unknown policy client {name!r}, use one of {sorted(BUILTIN_CLIENTS)}, an installed entry point of {ENTRY_POINT_GROUP} or module:Class"
            raise ValueError(msg)
        cls = matches[0].load()
        return _check_client_class(cls, name)
    module_name, _, attr = name.partition(":")
    obj: Any = importlib.import_module(module_name)
    for part in attr.split("."):
        obj = getattr(obj, part)
    return _check_client_class(obj, name)


def _check_client_class(cls: Any, name: str) -> type[PolicyClient]:
    if not (isinstance(cls, type) and issubclass(cls, PolicyClient)):
        msg = f"{name} is not a PolicyClient"
        raise TypeError(msg)
    return cls


def available_policy_clients() -> list[str]:
    return sorted([*BUILTIN_CLIENTS, *(ep.name for ep in entry_points(group=ENTRY_POINT_GROUP))])


# ---------------------------------------------------------------------------------------------------- executor


@dataclass
class _Chunk:
    chunk_id: int
    request: ChunkRequest
    chunk: ActionChunk
    sent: float
    received: float
    switch_step: int = -1


@dataclass(kw_only=True)
class StepSource:
    """Where the action of one step came from, recorded per step in the ``annotations`` column."""

    chunk_id: int = -1
    chunk_index: int = -1
    request_id: int = -1
    request_sent: float = float("nan")
    response_received: float = float("nan")
    switch_step: int = -1
    """Episode step at which this chunk became the active chunk."""
    waited: float = 0.0
    """Seconds the robot loop waited for the server in this step (0 when the chunk was ready)."""

    def as_dict(self) -> dict[str, Any]:
        return dict(self.__dict__)


class ChunkExecutor:
    """Turns a policy client into one action per env step.

    Without RTC (default): the first ``execute_steps`` actions of a chunk are executed, then the server is
    queried synchronously with the newest observation.

    With RTC (``rtc_delay > 0``): ``rtc_delay`` steps before the switch-over step (after ``execute_steps``
    actions of the current chunk) a query with the prefix and the guidance is sent from a worker thread while the
    loop keeps executing the current chunk, see the module docstring.
    """

    def __init__(self, client: PolicyClient, *, execute_steps: int | None = None, rtc_delay: int = 0):
        if rtc_delay > 0 and not client.capabilities.realtime_chunking:
            msg = f"{type(client).__name__} does not support real-time chunking, set the RTC delay to 0"
            raise ValueError(msg)
        if rtc_delay < 0:
            msg = "rtc_delay must be >= 0"
            raise ValueError(msg)
        self.client = client
        self.execute_steps = execute_steps
        self.rtc_delay = rtc_delay
        self._ids = itertools.count()
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="policy")
        self._lock = threading.Lock()
        self._current: _Chunk | None = None
        self._pending: tuple[ChunkRequest, float, Future] | None = None
        self.instruction = ""

    @property
    def rtc(self) -> bool:
        return self.rtc_delay > 0

    def reset(self, instruction: str):
        self.cancel()
        self.instruction = instruction
        self.client.reset(instruction)

    def cancel(self):
        """Drops the current chunk and any query in flight (its answer is ignored)."""
        self._current = None
        self._pending = None

    def close(self):
        self.cancel()
        self._pool.shutdown(wait=False, cancel_futures=True)

    def _horizon(self, chunk: _Chunk) -> int:
        n = len(chunk.chunk.actions)
        return n if self.execute_steps is None else min(self.execute_steps, n)

    def _send(
        self, obs: dict[str, Any], episode_step: int, current: _Chunk | None
    ) -> tuple[ChunkRequest, float, Future]:
        request = ChunkRequest(
            request_id=next(self._ids), observation=obs, instruction=self.instruction, episode_step=episode_step
        )
        if self.rtc and current is not None and current.chunk.raw is not None:
            idx = episode_step - current.request.episode_step
            raw = np.asarray(current.chunk.raw)
            request.prefix = raw[idx : idx + self.rtc_delay]
            request.guidance = raw[idx + self.rtc_delay :]
            request.delay = len(request.prefix)
            request.prefix_chunk_id = current.request.request_id
            request.prefix_start = idx
        sent = time.time()
        return request, sent, self._pool.submit(self.client.infer, request)

    def _take(self, pending: tuple[ChunkRequest, float, Future]) -> _Chunk:
        request, sent, future = pending
        chunk = future.result()
        if not chunk.actions:
            msg = "the policy server returned an empty chunk"
            raise RuntimeError(msg)
        return _Chunk(chunk_id=request.request_id, request=request, chunk=chunk, sent=sent, received=time.time())

    def act(self, obs: dict[str, Any], episode_step: int) -> tuple[dict[str, Any], StepSource, bool]:
        """Returns the action for this step, where it came from and whether the policy reported done."""
        waited = 0.0
        current = self._current
        if current is not None:
            idx = episode_step - current.request.episode_step
            horizon = self._horizon(current)
            if self.rtc:
                # send the query delay steps before the switch-over step
                if self._pending is None and idx >= max(horizon - self.rtc_delay, 0):
                    self._pending = self._send(obs, episode_step, current)
                # switch to the newest answer once its switch-over step is reached
                if self._pending is not None and self._pending[2].done():
                    pending_idx = episode_step - self._pending[0].episode_step
                    if pending_idx >= self._pending[0].delay or idx >= len(current.chunk.actions):
                        current = self._take(self._pending)
                        current.switch_step = episode_step
                        self._pending = None
                if current is self._current and idx >= len(current.chunk.actions):
                    # starved: the old chunk is used up and the answer is not there yet, wait for it
                    t0 = time.time()
                    pending = self._pending or self._send(obs, episode_step, None)
                    current = self._take(pending)
                    current.switch_step = episode_step
                    self._pending = None
                    waited = time.time() - t0
            elif idx >= horizon:
                current = None
        if current is None:
            t0 = time.time()
            current = self._take(self._send(obs, episode_step, None))
            current.switch_step = episode_step
            waited = time.time() - t0
        self._current = current
        idx = episode_step - current.request.episode_step
        idx = min(idx, len(current.chunk.actions) - 1)
        source = StepSource(
            chunk_id=current.chunk_id,
            chunk_index=idx,
            request_id=current.request.request_id,
            request_sent=current.sent,
            response_received=current.received,
            switch_step=current.switch_step,
            waited=waited,
        )
        return current.chunk.actions[idx], source, current.chunk.done


# ---------------------------------------------------------------------------------------------------- clients


class MockPolicyClient(PolicyClient):
    """Test client: answers after ``delay_s`` seconds with a chunk of ``horizon`` hold actions.

    ``raw[j] = [request_id, j]`` makes the alignment of chunks and RTC prefixes checkable. Options:
    ``delay_s`` (float), ``horizon`` (int), ``robots`` (list of robot names), ``rtc`` (bool, capability).
    """

    capabilities = PolicyCapabilities(realtime_chunking=True)

    def __init__(self, host: str = "mock", port: int = 0, **options: Any):
        super().__init__(host, port, **options)
        self.delay_s = float(options.get("delay_s", 0.0))
        self.horizon = int(options.get("horizon", 8))
        self.requests: list[ChunkRequest] = []
        self.connected = False

    def connect(self) -> None:
        self.connected = True

    def infer(self, request: ChunkRequest) -> ActionChunk:
        self.requests.append(request)
        time.sleep(self.delay_s)
        robots = [k for k in request.observation if k != "frames"]
        actions = []
        for _ in range(self.horizon):
            act: dict[str, Any] = {}
            for robot in robots:
                act[robot] = {"tquat": np.array([0, 0, 0, 0, 0, 0, 1.0])}
                if "gripper" in request.observation[robot]:
                    act[robot]["gripper"] = np.asarray(request.observation[robot]["gripper"], dtype=np.float32)
            actions.append(act)
        raw = np.array([[request.request_id, j] for j in range(self.horizon)], dtype=np.float64)
        if request.prefix is not None:
            # like an RTC server: the committed prefix is kept as is
            raw[: request.delay] = request.prefix
        return ActionChunk(actions=actions, raw=raw, metadata={"server_time": self.delay_s})


class VLAgentsPolicyClient(PolicyClient):
    """Client of a `vlagents <https://github.com/RobotControlStack/vlagents>`_ policy server, following
    ``examples/inference/franka.py``: cameras, joints, gripper and pose per robot are sent, the answer is one
    absolute joint position + gripper action per robot (env with ``ControlMode.JOINTS`` and ``RelativeTo.NONE``).

    No real-time chunking. Options: ``model`` (default ``lerobot``), ``on_same_machine``, ``jpeg_encoding``,
    ``image_size`` (``[w, h]`` or None), ``robots`` (default: all robots of the observation).
    """

    capabilities = PolicyCapabilities(realtime_chunking=False)

    def __init__(self, host: str, port: int, **options: Any):
        super().__init__(host, port, **options)
        self._agent: Any = None

    def connect(self) -> None:
        try:
            from vlagents.client import RemoteAgent
        except ImportError as e:
            msg = "the vlagents client is not installed: pip install rcs_webui[eval]"
            raise ImportError(msg) from e
        image_size = self.options.get("image_size", (224, 224))
        self._agent = RemoteAgent(
            self.host,
            self.port,
            self.options.get("model", "lerobot"),
            bool(self.options.get("on_same_machine", False)),
            bool(self.options.get("jpeg_encoding", True)),
            tuple(image_size) if image_size else None,
        )
        self._agent.ensure_connected()

    def infer(self, request: ChunkRequest) -> ActionChunk:
        from vlagents.policies.interface import Obs, SingleObs

        if self._agent is None:
            self.connect()
        assert self._agent is not None
        obs = request.observation
        robots = self.options.get("robots") or [k for k in obs if k != "frames"]
        cameras = {name: frame["rgb"]["data"] for name, frame in (obs.get("frames") or {}).items()}
        per_robot = {
            robot: SingleObs(
                cameras=cameras,
                joints=np.asarray(obs[robot]["joints"], dtype=np.float32),
                gripper=float(np.asarray(obs[robot].get("gripper", [1.0])).reshape(-1)[0]),
                xyzrpy=np.asarray(obs[robot]["xyzrpy"], dtype=np.float32) if "xyzrpy" in obs[robot] else None,
                tquat=np.asarray(obs[robot]["tquat"], dtype=np.float32) if "tquat" in obs[robot] else None,
            )
            for robot in robots
        }
        result = self._agent.act(Obs(obs=per_robot, language_instruction=request.instruction))
        actions = []
        raw_rows = []
        done = False
        for step in result.acts:
            act = {}
            row: list[float] = []
            for robot in robots:
                single = step[robot]
                joints = np.asarray(single.action, dtype=np.float64)
                gripper = np.asarray([single.gripper], dtype=np.float32)
                act[robot] = {"joints": joints, "gripper": gripper}
                row += [*joints.tolist(), float(gripper[0])]
                done = done or bool(getattr(single, "done", False))
            actions.append(act)
            raw_rows.append(row)
        return ActionChunk(actions=actions, raw=np.asarray(raw_rows), done=done)

    def close(self) -> None:
        self._agent = None


BUILTIN_CLIENTS = {
    "mock": f"{__name__}:MockPolicyClient",
    "vlagents": f"{__name__}:VLAgentsPolicyClient",
}
