import copy
import logging
import threading
from dataclasses import dataclass, field

try:
    import evdev
    from evdev import ecodes

    HAS_EVDEV = True
except ImportError:
    HAS_EVDEV = False

from rcs.envs.base import ArmWithGripper, ControlMode, RelativeTo
from rcs.operator.interface import BaseOperator, BaseOperatorConfig, TeleopCommands
from rcs.sim.sim import Sim
from rcs.utils import SimpleFrameRate

logger = logging.getLogger(__name__)

_SUPPORTED_COMMANDS = frozenset({"record", "success", "failure", "sync_position"})


class FootPedalOperator(BaseOperator):
    """Command-only operator for foot pedals exposed as a Linux evdev input device."""

    control_mode = (ControlMode.JOINTS, RelativeTo.NONE)

    def __init__(self, config: "FootPedalOperatorConfig", sim: Sim | None = None):
        super().__init__(config, sim)
        self.config: FootPedalOperatorConfig
        self._cmd_lock = threading.Lock()
        self._exit_requested = False
        self._commands = TeleopCommands()
        self.control_mode = self.config.control_mode
        self.controller_names = []
        self._device: evdev.InputDevice | None = None
        self._device_thread: threading.Thread | None = None

        invalid_commands = set(self.config.command_bindings.values()) - _SUPPORTED_COMMANDS
        if invalid_commands:
            msg = (
                "Unsupported foot pedal command bindings: "
                f"{sorted(invalid_commands)}. Supported commands are {sorted(_SUPPORTED_COMMANDS)}."
            )
            raise ValueError(msg)

        if not HAS_EVDEV:
            msg = "evdev is not installed. Install it to use FootPedalOperator."
            raise ImportError(msg)

        device_path = self._find_device_path(self.config.device_name_substring)
        if device_path is None:
            msg = f"Could not find a foot pedal input device matching '{self.config.device_name_substring}'."
            raise FileNotFoundError(msg)

        self._device = evdev.InputDevice(device_path)
        if self.config.grab_device:
            self._device.grab()

        self._device_thread = threading.Thread(target=self._evdev_read_loop, daemon=True)
        self._device_thread.start()
        logger.info(f"Connected foot pedal device {self._device.name} at {device_path}")

    def _find_device_path(self, substring: str) -> str | None:
        for path in evdev.list_devices():
            device = evdev.InputDevice(path)
            if substring.lower() in device.name.lower():
                return path
        return None

    def _trigger_command(self, command_name: str):
        with self._cmd_lock:
            setattr(self._commands, command_name, True)

    def _evdev_read_loop(self):
        assert self._device is not None
        try:
            for event in self._device.read_loop():
                if self._exit_requested:
                    break
                if event.type != ecodes.EV_KEY or event.value not in (1, 2):
                    continue

                key_name = getattr(event, "keycode", None)
                if isinstance(key_name, list):
                    key_name = key_name[0]

                command_name = self.config.command_bindings.get(str(key_name))
                if command_name is not None:
                    self._trigger_command(command_name)
        except OSError:
            if not self._exit_requested:
                logger.warning("Foot pedal device disconnected.", exc_info=True)

    def consume_commands(self) -> TeleopCommands:
        with self._cmd_lock:
            cmds = copy.copy(self._commands)
            self._commands = TeleopCommands()
            return cmds

    def reset_operator_state(self):
        pass

    def consume_action(self) -> dict[str, ArmWithGripper]:
        return {}

    def run(self):
        rate_limiter = SimpleFrameRate(self.config.read_frequency, "foot pedal operator")
        while not self._exit_requested:
            rate_limiter()

    def close(self):
        self._exit_requested = True

        if self._device is not None:
            try:
                if self.config.grab_device:
                    self._device.ungrab()
                self._device.close()
            except OSError:
                pass

        if self._device_thread is not None and self._device_thread.is_alive():
            self._device_thread.join(timeout=1.0)

        if self.is_alive() and threading.current_thread() != self:
            self.join(timeout=1.0)


@dataclass(kw_only=True)
class FootPedalOperatorConfig(BaseOperatorConfig):
    operator_class: type[BaseOperator] = field(default=FootPedalOperator)
    control_mode: tuple[ControlMode, RelativeTo] = (ControlMode.JOINTS, RelativeTo.NONE)
    device_name_substring: str = "Foot Switch"
    grab_device: bool = True
    command_bindings: dict[str, str] = field(
        default_factory=lambda: {
            "KEY_B": "sync_position",
            "KEY_C": "record",
            "KEY_A": "failure",
        }
    )
