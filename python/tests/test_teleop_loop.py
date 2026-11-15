from typing import Any

import numpy as np
from rcs.envs.base import ControlMode, RelativeTo
from rcs.operator.interface import BaseOperator, TeleopCommands, TeleopLoop


class _Operator(BaseOperator):
    control_mode = (ControlMode.CARTESIAN_TQuat, RelativeTo.LAST_STEP)
    controller_names = ["right"]  # noqa: RUF012

    def __init__(self):
        super().__init__(config=None)  # type: ignore[arg-type]
        self.resets = 0

    def consume_commands(self) -> TeleopCommands:
        return TeleopCommands()

    def consume_action(self) -> dict[str, Any]:
        return {"right": {"tquat": np.array([0, 0, 0, 0, 0, 0, 1.0]), "gripper": np.array([1.0])}}

    def reset_operator_state(self):
        self.resets += 1


class _Env:
    """Truncates the second step like a reflex on the hardware would do."""

    def __init__(self):
        self.steps = 0
        self.resets = 0
        self.loop: TeleopLoop | None = None

    def get_wrapper_attr(self, name: str):
        assert name == "envs"
        return {"right": None}

    def reset(self):
        self.resets += 1
        return {"right": {"joints": np.zeros(7)}}, {}

    def step(self, _action):
        self.steps += 1
        if self.steps >= 3:
            assert self.loop is not None
            self.loop._exit_requested = True
        truncated = self.steps == 2
        return {"right": {"joints": np.zeros(7)}}, 0.0, False, truncated, {"robot_error": "reflex" if truncated else ""}


def test_teleop_loop_resets_on_truncation():
    env = _Env()
    operator = _Operator()
    loop = TeleopLoop(env, operator)  # type: ignore[arg-type]
    env.loop = loop
    loop.environment_step_loop()
    # initial reset and one reset because of the truncated step
    assert env.resets == 2
    assert operator.resets == 1
