import contextlib
import logging
import threading
import typing

from rcs._core.common import Gripper, GripperConfig, GripperState
from rcs.common_typing import GripperConfigKwargs
from robotiq2f import Robotiq2F85, Robotiq2FStatus

import rcs

logger = logging.getLogger(__name__)


class RobotiQ2F85GripperConfig(GripperConfig):

    def __init__(
        self,
        serial_number: str,
        speed: float = 100,
        force: float = 50,
        async_control: bool = True,
        **kwargs: typing.Unpack[GripperConfigKwargs],
    ) -> None:
        """
        Args:
            serial_number: Get the serial number with `udevadm info -a -n /dev/ttyUSB0 | grep serial`, make sure you have read/write permissions to the port.
            speed: Speed in mm/s. Must be between 20 and 150 mm/s.
            force: Force in N. Must be between 20 and 235 N.
            async_control: If True, gripper commands return immediately without waiting for the movement to complete
                (a new command interrupts any ongoing movement), and the driver polls the gripper status in a
                background thread so that state reads are served from cache instead of blocking on Modbus.
        """
        super().__init__(**kwargs)
        self.serial_number = serial_number
        self.speed = speed
        self.force = force
        self.async_control = async_control
        self.gripper_type = rcs.common.GripperType.Robotiq2F85


class RobotiQ2F85GripperState(GripperState):
    def __init__(self, state: Robotiq2FStatus) -> None:
        super().__init__()
        self.state = state


class RobotiQ2F85Gripper(Gripper):
    def __init__(self, cfg: RobotiQ2F85GripperConfig):
        super().__init__()
        self._cfg: RobotiQ2F85GripperConfig = cfg
        self.gripper = Robotiq2F85(serial_number=cfg.serial_number, async_control=cfg.async_control)
        self.gripper.reset()
        # async control: robotiq2f only reads the status in a thread, go_to() still writes the Modbus registers in
        # the calling thread and waits for the gripper's reply (seen: 80-200 ms stalls of the env step at open /
        # close). Commands are therefore sent from a writer thread; only the newest pending command is sent.
        self._command = threading.Condition()
        self._pending: tuple[float, float, float] | None = None
        self._stop_writer = False
        self._writer: threading.Thread | None = None
        if cfg.async_control:
            self._writer = threading.Thread(target=self._write_commands, daemon=True, name="robotiq-commands")
            self._writer.start()

    def get_normalized_width(self) -> float:
        return self.gripper.opening / self.gripper.MAX_OPENING

    def grasp(self) -> None:
        """
        Close the gripper to grasp an object.
        """
        self.set_normalized_width(0.0, force=self._cfg.force)

    def open(self) -> None:
        """
        Open the gripper to its maximum width.
        """
        self.set_normalized_width(1.0)

    def reset(self) -> None:
        self.open()

    def set_normalized_width(self, width: float, force: float = 0) -> None:
        """
        Set the gripper width to a normalized value between 0 and 1.
        """
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        command = (
            float(width * self.gripper.MAX_OPENING),
            float(self._cfg.speed),
            float(force if force != 0 else self._cfg.force),
        )
        if self._writer is None:
            self.gripper.go_to(opening=command[0], speed=command[1], force=command[2])
            return
        with self._command:
            self._pending = command
            self._command.notify()

    def _write_commands(self) -> None:
        while True:
            with self._command:
                while self._pending is None and not self._stop_writer:
                    self._command.wait()
                if self._stop_writer:
                    return
                opening, speed, force = self._pending  # type: ignore[misc]
                self._pending = None
            try:
                self.gripper.go_to(opening=opening, speed=speed, force=force)
            except Exception:
                logger.exception("sending the Robotiq command failed")

    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.set_normalized_width(0.0)

    def close(self) -> None:
        """Stops the command writer and the status thread of async control, then closes the serial port.

        robotiq2f 0.2.0 reads the status in a ``while True`` thread without a stop flag, so closing the port under
        it made the thread crash with ``termios.error: Bad file descriptor`` (seen when the RCS web interface
        closed the env). The thread looks ``read_status_sync`` up on the instance in every loop: shadowing it with
        a function that raises SystemExit ends the thread silently after its current read. Remove this once
        robotiq2f can stop the thread itself.
        """
        writer = getattr(self, "_writer", None)
        if writer is not None:
            with self._command:
                self._stop_writer = True
                self._command.notify()
            writer.join(timeout=2.0)
        gripper = self.gripper
        thread = getattr(gripper, "_async_thread", None)
        if thread is not None and thread.is_alive():

            def _stop_status_thread():
                raise SystemExit

            gripper.read_status_sync = _stop_status_thread  # type: ignore[method-assign]
            thread.join(timeout=2.0)
            if thread.is_alive():
                logger.warning("the Robotiq status thread did not stop, closing the serial port anyway")
        lock = getattr(gripper, "_client_mutex", None)
        with lock if lock is not None else contextlib.nullcontext():
            gripper.client.serial.close()

    def get_config(self) -> GripperConfig:
        return self._cfg

    def set_config(self, cfg: RobotiQ2F85GripperConfig) -> None:
        self._cfg = cfg

    def get_state(self) -> GripperState:
        return RobotiQ2F85GripperState(state=self.gripper.read_status())
