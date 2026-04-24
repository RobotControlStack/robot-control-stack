import typing

from rcs._core.common import Gripper, GripperConfig, GripperState
from rcs.common_typing import GripperConfigKwargs
from Robotiq2F85Driver.Robotiq2F85Driver import GripperStatus, Robotiq2F85Driver


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


class RobotiQ2F85GripperState(GripperState):
    def __init__(self, state: GripperStatus) -> None:
        super().__init__()
        self.state = state


class RobotiQ2F85Gripper(Gripper):
    def __init__(self, cfg: RobotiQ2F85GripperConfig):
        super().__init__()
        self._cfg: RobotiQ2F85GripperConfig = cfg
        self.gripper = Robotiq2F85Driver(serial_number=cfg.serial_number)

    def get_normalized_width(self) -> float:
        # value between 0 and 1 (0 is closed)
        return self.gripper.opening / 85

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
        self.gripper.reset()

    def set_normalized_width(self, width: float, force: float = 0) -> None:
        """
        Set the gripper width to a normalized value between 0 and 1.
        """
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        abs_width = width * 85
        self.gripper.go_to(
            opening=float(abs_width),
            speed=self._cfg.speed,
            force=force if force != 0 else self._cfg.force,
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
