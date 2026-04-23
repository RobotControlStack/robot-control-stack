import copy
import logging
import threading
from dataclasses import dataclass, field

try:
    from pynput import keyboard

    HAS_PYNPUT = True
except ImportError:
    HAS_PYNPUT = False

from rcs.envs.base import ControlMode, RelativeTo
from rcs.operator.interface import BaseOperator, BaseOperatorConfig, TeleopCommands
from rcs.sim.sim import Sim
from rcs.utils import SimpleFrameRate

logger = logging.getLogger(__name__)


class KeyboardOperator(BaseOperator):
    """Keyboard-only operator that emits teleop commands and no motion actions."""

    control_mode = (ControlMode.JOINTS, RelativeTo.NONE)

    def __init__(self, config: "KeyboardOperatorConfig", sim: Sim | None = None):
        super().__init__(config, sim)
        self.config: KeyboardOperatorConfig
        self._cmd_lock = threading.Lock()
        self._exit_requested = False
        self._commands = TeleopCommands()
        self.control_mode = self.config.control_mode
        self.controller_names = []

        if HAS_PYNPUT:
            self._listener = keyboard.Listener(on_press=self._on_press)
            self._listener.start()
        else:
            logger.warning("pynput not found. Keyboard commands disabled.")

    def _on_press(self, key):
        try:
            if key == keyboard.Key.space:
                with self._cmd_lock:
                    self._commands.record = True
                return

            char = key.char
        except AttributeError:
            return

        with self._cmd_lock:
            if char == self.config.sync_key:
                self._commands.sync_position = True
            elif char == self.config.record_key:
                self._commands.record = True
            elif char == self.config.success_key:
                self._commands.success = True
            elif char == self.config.failure_key:
                self._commands.failure = True

    def consume_commands(self) -> TeleopCommands:
        with self._cmd_lock:
            cmds = copy.copy(self._commands)
            self._commands = TeleopCommands()
            return cmds

    def reset_operator_state(self):
        pass

    def consume_action(self) -> dict[str, object]:
        return {}

    def run(self):
        rate_limiter = SimpleFrameRate(self.config.read_frequency, "keyboard operator")
        while not self._exit_requested:
            rate_limiter()

    def close(self):
        self._exit_requested = True
        if HAS_PYNPUT and hasattr(self, "_listener"):
            self._listener.stop()
        if self.is_alive() and threading.current_thread() != self:
            self.join(timeout=1.0)


@dataclass(kw_only=True)
class KeyboardOperatorConfig(BaseOperatorConfig):
    operator_class: type[BaseOperator] = field(default=KeyboardOperator)
    control_mode: tuple[ControlMode, RelativeTo] = (ControlMode.JOINTS, RelativeTo.NONE)
    sync_key: str = "s"
    record_key: str = " "
    success_key: str = "x"
    failure_key: str = "r"
