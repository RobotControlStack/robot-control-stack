import typing

from rcs._core.common import Gripper, GripperConfig, GripperState
from rcs.common_typing import GripperConfigKwargs
from Robotiq2F85Driver.Robotiq2F85Driver import GripperStatus, Robotiq2F85Driver

import rcs


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
            async_control: If True, gripper commands return immediately without waiting for the movement to complete. A new command interrupts any ongoing movement.
        """
        super().__init__(**kwargs)
        self.serial_number = serial_number
        self.speed = speed
        self.force = force
        self.async_control = async_control
        self.gripper_type = rcs.common.GripperType("Robotiq2F85")


class RobotiQ2F85GripperState(GripperState):
    def __init__(self, state: GripperStatus) -> None:
        super().__init__()
        self.state = state


class RobotiQ2F85Gripper(Gripper):
    MIN_FORCE = 20.0

    def __init__(self, cfg: RobotiQ2F85GripperConfig):
        super().__init__()
        self._cfg: RobotiQ2F85GripperConfig = cfg
        self.gripper = Robotiq2F85Driver(serial_number=cfg.serial_number)
        self._last_normalized_width = 1.0
        self.gripper.reset()

    def get_normalized_width(self) -> float:
        """Return the measured, normalized finger opening.

        ``read_status`` is cached by ``Robotiq2F85Driver`` (30 Hz by default),
        so this exposes the physical state to ``GripperWrapper(binary=False)``
        without issuing a Modbus transaction on every environment step.
        """
        opening = self.gripper.read_status().opening
        return float(min(max(opening / 85.0, 0.0), 1.0))

    def grasp(self) -> None:
        """
        Close the gripper to grasp an object.
        """
        self.set_normalized_width(0.0, force=1.0)

    def open(self) -> None:
        """
        Open the gripper to its maximum width.
        """
        self.set_normalized_width(1.0)

    def reset(self) -> None:
        self.open()

    def set_normalized_width(self, width: float, force: float = 0) -> None:
        """
        Set the gripper width and force to normalized values between 0 and 1.
        """
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        if not (0 <= force <= 1):
            msg = f"Force must be between 0 and 1, got {force}."
            raise ValueError(msg)
        self._last_normalized_width = width
        abs_width = width * 85
        if self._cfg.enable_force_action:
            max_force = max(self.MIN_FORCE, self._cfg.force)
            abs_force = self.MIN_FORCE + force * (max_force - self.MIN_FORCE)
        else:
            abs_force = self._cfg.force
        self.gripper.go_to(
            opening=float(abs_width),
            speed=self._cfg.speed,
            force=abs_force,
            blocking_call=not self._cfg.async_control,
        )

    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.set_normalized_width(0.0)

    def close(self) -> None:
        self.gripper.client.serial.close()

    def get_config(self) -> GripperConfig:
        return self._cfg

    def set_config(self, cfg: RobotiQ2F85GripperConfig) -> None:
        self._cfg = cfg

    def get_state(self) -> GripperState:
        return RobotiQ2F85GripperState(state=self.gripper.read_status())
