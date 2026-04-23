import logging
import threading
from dataclasses import dataclass, field

from rcs.operator.interface import BaseOperator, BaseOperatorConfig, TeleopCommands
from rcs.sim.sim import Sim
from rcs.utils import SimpleFrameRate

logger = logging.getLogger(__name__)


class ComposeOperator(BaseOperator):
    """Compose two operators so the override action wins on overlapping controllers."""

    def __init__(self, config: "ComposeOperatorConfig", sim: Sim | None = None):
        super().__init__(config, sim)
        self.config: ComposeOperatorConfig
        self._exit_requested = False
        self._child_lock = threading.Lock()

        self._base_operator = self.config.base.operator_class(self.config.base, sim)
        self._override_operator = self.config.override.operator_class(self.config.override, sim)

        if self._base_operator.control_mode != self._override_operator.control_mode:
            msg = (
                "ComposeOperator requires both child operators to use the same control_mode. "
                f"Got base={self._base_operator.control_mode} and "
                f"override={self._override_operator.control_mode}."
            )
            raise ValueError(msg)

        self.control_mode = self._base_operator.control_mode
        self.controller_names = list(
            dict.fromkeys(self._base_operator.controller_names + self._override_operator.controller_names)
        )

    def consume_commands(self) -> TeleopCommands:
        with self._child_lock:
            base_commands = self._base_operator.consume_commands()
            override_commands = self._override_operator.consume_commands()
        return TeleopCommands.merged(base_commands, override_commands)

    def reset_operator_state(self):
        with self._child_lock:
            self._base_operator.reset_operator_state()
            self._override_operator.reset_operator_state()

    def consume_action(self):
        with self._child_lock:
            actions = self._base_operator.consume_action()
            override_actions = self._override_operator.consume_action()
        return actions | override_actions

    def run(self):
        self._base_operator.start()
        self._override_operator.start()

        rate_limiter = SimpleFrameRate(self.config.read_frequency, "compose operator")

        try:
            while not self._exit_requested:
                if not self._base_operator.is_alive():
                    logger.warning("ComposeOperator base child stopped.")
                    break
                if not self._override_operator.is_alive():
                    logger.warning("ComposeOperator override child stopped.")
                    break
                rate_limiter()
        finally:
            self.close()

    def close(self):
        self._exit_requested = True
        self._base_operator.close()
        self._override_operator.close()

        current_thread = threading.current_thread()
        for operator in (self._base_operator, self._override_operator):
            if operator.is_alive() and current_thread != operator:
                operator.join(timeout=1.0)


@dataclass(kw_only=True)
class ComposeOperatorConfig(BaseOperatorConfig):
    operator_class: type[BaseOperator] = field(default=ComposeOperator)
    base: BaseOperatorConfig
    override: BaseOperatorConfig
