import logging
from ctypes import c_bool, c_double, c_int
from dataclasses import dataclass, field
from enum import IntFlag, auto
from functools import cached_property
from itertools import chain
from multiprocessing import Array, Process, RLock, Value
from multiprocessing.sharedctypes import Synchronized, SynchronizedArray
from multiprocessing.synchronize import RLock as RLockType
from socket import AF_INET, SOCK_DGRAM, socket
from struct import unpack
from threading import Event, Thread

import numpy as np
import rcsss
from numpy.typing import NDArray
from rcsss._core.common import Pose
from rcsss.envs.base import ControlMode, FR3Env, RelativeActionSpace
from rcsss.envs.sim import FR3Sim
from rcsss.sim import FR3, FR3Config, Sim

logger = logging.getLogger(__name__)


class Button(IntFlag):
    L_SQUEEZE = auto()
    L_TRIGGER = auto()
    R_SQUEEZE = auto()
    R_TRIGGER = auto()


@dataclass(slots=True)
class ViveControllerState:
    last_controller_pose_initialized: bool = False
    last_controller_pose: Pose = field(default_factory=Pose)
    curr_controller_pose: Pose = field(default_factory=Pose)


class UDPViveActionServer:
    # seven doubles and one integer in network byte order
    FMT = "!" + 7 * "d" + "i"

    def __init__(self, host: str, port: int):
        self._host: str = host
        self._port: int = port
        self._server_proc: Process | None = None
        self._lock: RLockType = RLock()
        self._pose_raw = Array(c_double, 7, lock=False)
        self._buttons = Value(c_int, 0, lock=False)
        self._exit_requested = Value(c_bool, False, lock=False)
        self._controller_state = ViveControllerState()

    @cached_property
    def _pose_raw_as_array(self) -> NDArray:
        return np.ctypeslib.as_array(self._pose_raw[:])

    @property
    def running(self) -> bool:
        return self._server_proc is not None and self._server_proc.is_alive()

    def next_action(self) -> Pose:
        if not self.running:
            msg = "Server is not running."
            raise RuntimeError(msg)
        while True:
            with self._lock:
                if not Button(int(self._buttons.value)) & Button.R_SQUEEZE:
                    self._controller_state.last_controller_pose_initialized = False
                    continue
                # Trigger is pressed
                if not self._controller_state.last_controller_pose_initialized:
                    self._controller_state.last_controller_pose = Pose(
                        quaternion=self._pose_raw_as_array[:4], translation=self._pose_raw_as_array[4:]
                    )
                    continue
                # Trigger is pressed and first pose since press is recorded
                self._controller_state.curr_controller_pose = Pose(
                    quaternion=self._pose_raw_as_array[:4], translation=self._pose_raw_as_array[4:]
                )
                displacement = (
                    self._controller_state.curr_controller_pose * self._controller_state.last_controller_pose.inverse()
                )
                self._controller_state.last_controller_pose = self._controller_state.curr_controller_pose
                return displacement

    @staticmethod
    def worker(
        shared_array: "SynchronizedArray[c_double]",
        shared_int: "Synchronized[c_int]",
        exit_requested: "Synchronized[c_bool]",
        lock: RLockType,
        host: str,
        port: int,
    ):
        warning_raised = False
        with socket(AF_INET, SOCK_DGRAM) as sock:
            sock.settimeout(0.1)
            sock.bind((host, port))
            while not exit_requested.value:
                try:
                    unpacked = unpack(UDPViveActionServer.FMT, sock.recv(7 * 8 + 4))
                    if warning_raised:
                        logger.info("[UDP Server] connection reestablished")
                        warning_raised = False
                except TimeoutError as err:
                    if not warning_raised:
                        msg = "[UDP server] socket timeout (0.1s), waiting for packets"
                        raise RuntimeWarning(msg) from err
                        warning_raised = True
                    break
                with lock:
                    shared_array[:] = unpacked[:7]
                    shared_int.value = unpacked[7]

    def start_worker(self):
        if self.running:
            msg = "Server already running"
            raise RuntimeError(msg)
        self._lock = RLock()
        self._server_proc = Process(
            target=UDPViveActionServer.worker(
                self._pose_raw, self._buttons, self._exit_requested, self._lock, self._host, self._port  # type: ignore
            )
        )
        self._server_proc.start()

    def stop_worker(self):
        if not self.running:
            msg = "Server is not running"
            raise RuntimeError(msg)
        assert self._server_proc
        self._exit_requested.value = c_bool(True)
        try:
            self._server_proc.join(timeout=1.0)
        except TimeoutError as err:
            msg = "Could not join server process. Killing it."
            raise RuntimeWarning(msg) from err
            self._server_proc.kill()

    def __enter__(self):
        self.start_worker()
        return self

    def __exit__(self, *_):
        self.stop_worker()


def environment_step_loop(action_server: UDPViveActionServer, env: RelativeActionSpace, stop_requested: Event):
    # assert env.action_space is TQuartDictType
    while not stop_requested.is_set():
        displacement = action_server.next_action()
        action = {
            "tquart": np.fromiter(
                iter=chain(displacement.translation(), displacement.rotation_q()), dtype=np.float64, count=7
            )
        }
        env.action(action)


def main():
    if "lab" not in rcsss.scenes:
        "This pip package was not built with the UTN lab models, aborting."
        return
    host = "localhost"
    port = 54321
    simulation = Sim(rcsss.scenes["lab"])
    robot = FR3(simulation, "0", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    fr3_config = FR3Config()
    fr3_config.realtime = False
    fr3_config.tcp_offset = Pose(quaternion=np.array([0, 0, 0, 1]), translation=np.array([0, 0, 0.1034]))
    env = RelativeActionSpace(FR3Sim(FR3Env(robot, ControlMode.CARTESIAN_TQuart), simulation))
    env.reset()
    with UDPViveActionServer(host, port) as action_server:
        stop_event = Event()
        t = Thread(target=environment_step_loop, args=(action_server, env, stop_event))
        t.start()
        input("Press enter to exit: ")
        stop_event.set()
        try:
            t.join(5.0)
        except TimeoutError as err:
            msg = "Thread did not join after five seconds. Exiting anyway."
            raise RuntimeWarning(msg) from err


if __name__ == "__main__":
    main()
