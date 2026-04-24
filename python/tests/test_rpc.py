import copy
import multiprocessing
import os
import socket
import sys
import time
import traceback
from contextlib import suppress
from multiprocessing.context import ForkServerContext, SpawnContext
from typing import Optional, Type, Union

import gymnasium as gym
import pytest
from rcs.envs.base import (
    ControlMode,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
    SimEnv,
)
from rcs.envs.configs import EmptyWorldFR3
from rcs.envs.sim import RobotSimWrapper
from rcs.rpc.client import RcsClient
from rcs.rpc.server import RcsServer

import rcs
from rcs import sim

HOST = "127.0.0.1"


def build_rpc_env() -> gym.Env:
    scene = EmptyWorldFR3()
    cfg = copy.deepcopy(scene.config())
    cfg.control_mode = ControlMode.JOINTS
    cfg.gripper_cfgs = None
    cfg.gripper_offsets = None
    cfg.camera_cfgs = None
    cfg.camera_adds = None
    cfg.headless = True
    cfg.sim_cfg.realtime = False
    cfg.sim_cfg.async_control = False

    prefixed_cfg = scene.prefixed_cfg(cfg)
    robot_name = scene.lead_robot_name(prefixed_cfg)
    robot_cfg = prefixed_cfg.robot_cfgs[robot_name]
    mjmodel = scene.create_model(prefixed_cfg)
    simulation = sim.Sim(mjmodel, prefixed_cfg.sim_cfg)
    kinematic_model_path, attachment_site = scene.kinematics_cfg(prefixed_cfg)[robot_name]
    ik = rcs.common.Pin(
        kinematic_model_path,
        attachment_site,
    )

    env: gym.Env = SimEnv(simulation)
    robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
    env = RobotWrapper(env, robot, ControlMode.JOINTS)
    env = RobotSimWrapper(env)
    return RelativeActionSpace(env, max_mov=0.1, relative_to=RelativeTo.LAST_STEP)


def can_bind_tcp() -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, 0))
        return True
    except PermissionError:
        return False


def get_free_port() -> int:
    if not can_bind_tcp():
        pytest.skip("TCP sockets are not permitted in this test environment.")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, 0))
        return s.getsockname()[1]


def wait_for_port(
    host: str,
    port: int,
    timeout: float,
    server_proc: Optional[multiprocessing.Process] = None,
    err_q: Optional[multiprocessing.Queue] = None,
) -> None:
    start = time.time()
    last_exc = None
    while time.time() - start < timeout:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            try:
                if s.connect_ex((host, port)) == 0:
                    return
            except OSError as e:
                last_exc = e
        if server_proc is not None and not server_proc.is_alive():
            server_err = None
            if err_q is not None:
                with suppress(Exception):
                    server_err = err_q.get_nowait()
            msg = f"Server process exited early (exitcode={server_proc.exitcode})."
            if server_err:
                msg += f"\nServer traceback:\n{server_err}"
            raise RuntimeError(msg)
        time.sleep(0.2)
    server_err = None
    if err_q is not None:
        with suppress(Exception):
            server_err = err_q.get_nowait()
    msg = f"Timed out waiting for {host}:{port} to open."
    if last_exc:
        msg += f" Last socket error: {last_exc}"
    if server_proc is not None and not server_proc.is_alive():
        msg += f" Server exitcode={server_proc.exitcode}."
    if server_err:
        msg += f"\nServer traceback:\n{server_err}"
    raise TimeoutError(msg)


def run_server(host: str, port: int, err_q: multiprocessing.Queue) -> None:
    try:
        env = build_rpc_env()
        server = RcsServer(env, host=host, port=port)
        try:
            server.start()
        finally:
            while True:
                time.sleep(1)
    except Exception:
        tb = "".join(traceback.format_exception(*sys.exc_info()))
        with suppress(Exception):
            err_q.put(tb)
        sys.exit(1)


def _mp_context() -> Union[SpawnContext, ForkServerContext]:
    methods = multiprocessing.get_all_start_methods()
    if "spawn" in methods:
        return multiprocessing.get_context("spawn")
    if "forkserver" in methods:
        return multiprocessing.get_context("forkserver")

    msg = "No suitable multiprocessing context found."
    raise RuntimeError(msg)


def _external_server_from_env() -> tuple[str, int] | None:
    host = os.getenv("RCS_TEST_HOST")
    port = os.getenv("RCS_TEST_PORT")
    if host and port:
        try:
            return host, int(port)
        except ValueError:
            pass
    if os.getenv("RCS_TEST_REUSE_SERVER") == "1":
        return HOST, 50055
    return None


def test_run_server_starts_and_stops():
    if not can_bind_tcp():
        pytest.skip("TCP sockets are not permitted in this test environment.")
    ext = _external_server_from_env()
    if ext:
        pytest.skip("External server reuse enabled via env; skipping spawn test.")
    ctx = _mp_context()
    err_q = ctx.Queue()
    port = get_free_port()
    server_proc = ctx.Process(target=run_server, args=(HOST, port, err_q))
    server_proc.start()
    try:
        wait_for_port(HOST, port, timeout=120.0, server_proc=server_proc, err_q=err_q)  # type: ignore
        assert server_proc.is_alive(), "Server process did not start as expected."
    finally:
        if server_proc.is_alive():
            server_proc.terminate()
            server_proc.join(timeout=5)
    assert not server_proc.is_alive(), "Server process did not terminate as expected."


@pytest.mark.skipif(not can_bind_tcp(), reason="TCP sockets are not permitted in this test environment.")
class TestRcsClientServer:
    client: RcsClient
    host: str = HOST
    port: int = 0
    server_proc = None
    err_q: Optional[multiprocessing.Queue] = None

    @classmethod
    def setup_class(cls: Type["TestRcsClientServer"]):
        ext = _external_server_from_env()
        if ext:
            cls.host, cls.port = ext
            cls.server_proc = None
            cls.err_q = None
            wait_for_port(cls.host, cls.port, timeout=60.0)
            cls.client = RcsClient(host=cls.host, port=cls.port)
            return

        ctx = _mp_context()
        cls.err_q = ctx.Queue()
        cls.host, cls.port = HOST, get_free_port()
        cls.server_proc = ctx.Process(target=run_server, args=(cls.host, cls.port, cls.err_q))
        cls.server_proc.start()
        wait_for_port(cls.host, cls.port, timeout=180.0, server_proc=cls.server_proc, err_q=cls.err_q)  # type: ignore
        cls.client = RcsClient(host=cls.host, port=cls.port)

    @classmethod
    def teardown_class(cls: Type["TestRcsClientServer"]):
        try:
            if getattr(cls, "client", None):
                cls.client.close()
        finally:
            if getattr(cls, "server_proc", None) and cls.server_proc and cls.server_proc.is_alive():
                cls.server_proc.terminate()
                cls.server_proc.join(timeout=5)

    def test_reset(self):
        obs, info = self.client.reset()
        assert obs is not None, "reset did not return an observation"

    def test_step(self):
        self.client.reset()
        act = self.client.action_space.sample()
        step_result = self.client.step(act)
        assert isinstance(step_result, (tuple, list)), "step did not return a tuple or list"

    def test_get_obs(self):
        self.client.reset()
        obs2 = self.client.get_robot_obs()
        assert obs2 is not None, "get_obs did not return an observation"

    def test_unwrapped(self):
        _ = self.client.unwrapped

    def test_close(self):
        if self.client is not None:
            self.client.close()
        wait_for_port(
            self.__class__.host,
            self.__class__.port,
            timeout=15.0,
            server_proc=self.__class__.server_proc,  # type: ignore
            err_q=self.__class__.err_q,
        )
        self.__class__.client = RcsClient(host=self.__class__.host, port=self.__class__.port)
