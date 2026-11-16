"""close() must stop the async status thread of robotiq2f before the serial port is closed (no hardware needed)."""

import threading
import time

from rcs_robotiq2f85.hw import RobotiQ2F85Gripper
from robotiq2f import Robotiq2F85


class _Serial:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class _Client:
    def __init__(self):
        self.serial = _Serial()


def test_close_stops_the_status_thread_before_closing_the_port():
    lib = Robotiq2F85.__new__(Robotiq2F85)
    lib.client = _Client()
    lib._client_mutex = threading.Lock()
    lib.read_frequency = 200
    reads = []

    def read_status_sync():  # stands in for the register read: fails like termios once the port is closed
        with lib._client_mutex:
            if lib.client.serial.closed:
                msg = "(9, 'Bad file descriptor')"
                raise OSError(msg)
            reads.append(time.perf_counter())

    lib.read_status_sync = read_status_sync
    lib._async_thread = threading.Thread(target=lib.read_status_thread, daemon=True)  # the library's own loop
    errors = []
    previous_hook = threading.excepthook
    # the default hook ignores SystemExit (how close() ends the thread), everything else is a crash
    threading.excepthook = lambda args: None if args.exc_type is SystemExit else errors.append(args.exc_value)
    try:
        lib._async_thread.start()
        while len(reads) < 5:
            time.sleep(0.01)
        gripper = RobotiQ2F85Gripper.__new__(RobotiQ2F85Gripper)
        gripper.gripper = lib
        gripper.close()
        time.sleep(0.05)
    finally:
        threading.excepthook = previous_hook
    assert lib.client.serial.closed
    assert not lib._async_thread.is_alive()
    assert errors == []


def _gripper_with_slow_go_to(delay: float):
    """A RobotiQ2F85Gripper with async control whose library go_to() blocks like a slow Modbus reply."""
    from rcs_robotiq2f85.hw import RobotiQ2F85GripperConfig

    lib = Robotiq2F85.__new__(Robotiq2F85)
    lib.client = _Client()
    lib._client_mutex = threading.Lock()
    lib.MAX_OPENING = 85.0
    sent = []

    def go_to(opening, **_kwargs):
        time.sleep(delay)
        sent.append(opening)

    lib.go_to = go_to
    gripper = RobotiQ2F85Gripper.__new__(RobotiQ2F85Gripper)
    gripper._cfg = RobotiQ2F85GripperConfig(serial_number="X", async_control=True)
    gripper.gripper = lib
    gripper._command = threading.Condition()
    gripper._pending = None
    gripper._stop_writer = False
    gripper._writer = threading.Thread(target=gripper._write_commands, daemon=True)
    gripper._writer.start()
    return gripper, lib, sent


def test_async_commands_do_not_block_the_caller():
    gripper, lib, sent = _gripper_with_slow_go_to(0.2)
    start = time.perf_counter()
    gripper.open()
    assert time.perf_counter() - start < 0.02  # the env step does not wait for the gripper's reply
    while gripper._pending is not None:  # the writer picked up the first command and is sending it
        time.sleep(0.001)
    # while the first command is being written, newer ones replace each other: only the newest is sent
    gripper.grasp()
    gripper.set_normalized_width(0.5)
    time.sleep(0.6)
    assert sent == [85.0, 42.5]
    gripper.close()
    assert not gripper._writer.is_alive()
    assert lib.client.serial.closed
