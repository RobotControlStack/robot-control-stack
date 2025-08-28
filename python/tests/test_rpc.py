import multiprocessing
import time
import socket
import sys
import traceback
import os
import pytest
from typing import Optional  # Add this import at the top
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)
from rcs.envs.base import ControlMode, RelativeTo
from rcs.rpc.server import RcsServer
from rcs.rpc.client import RcsClient

HOST = "127.0.0.1"

def get_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, 0))
        return s.getsockname()[1]

def wait_for_port(
    host: str,
    port: int,
    timeout: float,
    server_proc: Optional[multiprocessing.Process] = None,
    err_q: Optional[multiprocessing.Queue] = None
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
        # If the server process died, surface its error immediately
        if server_proc is not None and not server_proc.is_alive():
            server_err = None
            if err_q is not None:
                try:
                    server_err = err_q.get_nowait()
                except Exception:
                    pass
            msg = f"Server process exited early (exitcode={server_proc.exitcode})."
            if server_err:
                msg += f"\nServer traceback:\n{server_err}"
            raise RuntimeError(msg)
        time.sleep(0.2)
    server_err = None
    if err_q is not None:
        try:
            server_err = err_q.get_nowait()
        except Exception:
            pass
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
        env = SimEnvCreator()(
            control_mode=ControlMode.JOINTS,
            collision_guard=False,
            robot_cfg=default_sim_robot_cfg(),
            gripper_cfg=default_sim_gripper_cfg(),
            # Disabled to avoid rendering problem in python subprocess.
            #cameras=default_mujoco_cameraset_cfg(), 
            max_relative_movement=0.1,
            relative_to=RelativeTo.LAST_STEP,
        )
        # Bind explicitly to IPv4 loopback
        server = RcsServer(env, host=host, port=port)
        try:
            server.start()
        finally:
            # If start returns (non-blocking implementation), keep process alive
            while True:
                time.sleep(1)
    except Exception:
        tb = "".join(traceback.format_exception(*sys.exc_info()))
        try:
            err_q.put(tb)
        except Exception:
            pass
        sys.exit(1)

def _mp_context() -> multiprocessing.context.BaseContext:
    # Prefer spawn to avoid fork-related issues with GL/MuJoCo/threaded libs
    methods = multiprocessing.get_all_start_methods()
    if "spawn" in methods:
        return multiprocessing.get_context("spawn")
    if "forkserver" in methods:
        return multiprocessing.get_context("forkserver")
    return multiprocessing.get_context(methods[0])

def _external_server_from_env() -> tuple[str, int] | None:
    # Set RCS_TEST_HOST and RCS_TEST_PORT to reuse an already running server.
    host = os.getenv("RCS_TEST_HOST")
    port = os.getenv("RCS_TEST_PORT")
    if host and port:
        try:
            return host, int(port)
        except ValueError:
            pass
    # Convenience: RCS_TEST_REUSE_SERVER=1 will use HOST + default port 50055
    if os.getenv("RCS_TEST_REUSE_SERVER") == "1":
        return HOST, 50055
    return None

def test_run_server_starts_and_stops():
    # Skip if reusing an external server
    ext = _external_server_from_env()
    if ext:
        pytest.skip("External server reuse enabled via env; skipping spawn test.")
    ctx = _mp_context()
    err_q = ctx.Queue()
    port = get_free_port()
    server_proc = ctx.Process(target=run_server, args=(HOST, port, err_q))
    server_proc.start()
    try:
        wait_for_port(HOST, port, timeout=120.0, server_proc=server_proc, err_q=err_q)
        assert server_proc.is_alive(), "Server process did not start as expected."
    finally:
        if server_proc.is_alive():
            server_proc.terminate()
            server_proc.join(timeout=5)
    assert not server_proc.is_alive(), "Server process did not terminate as expected."

class TestRcsClientServer:
    @classmethod
    def setup_class(cls):
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
        # Wait until the server is actually listening or fail early if it crashed
        wait_for_port(cls.host, cls.port, timeout=180.0, server_proc=cls.server_proc, err_q=cls.err_q)
        cls.client = RcsClient(host=cls.host, port=cls.port)

    @classmethod
    def teardown_class(cls):
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
        obs2 = self.client.get_obs()
        assert obs2 is not None, "get_obs did not return an observation"

    def test_unwrapped(self):
        _ = self.client.unwrapped

    def test_close(self):
        self.client.close()
        # Reconnect for further tests
        wait_for_port(self.__class__.host, self.__class__.port, timeout=15.0,
                      server_proc=self.__class__.server_proc, err_q=self.__class__.err_q)
        self.__class__.client = RcsClient(host=self.__class__.host, port=self.__class__.port)
