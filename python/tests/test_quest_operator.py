import threading
from typing import Any, cast

from rcs.operator.interface import TeleopCommands
from rcs.operator.quest import QuestOperator


class _Config:
    switched_left_right = True


def test_consume_commands_swaps_both_reset_origin_keys():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())
    operator._cmd_lock = threading.Lock()
    operator._commands = TeleopCommands(reset_origin_to_current={"left": True, "right": False})

    cmds = QuestOperator.consume_commands(operator)

    assert cmds.reset_origin_to_current == {"right": True, "left": False}
    assert operator._commands.reset_origin_to_current == {}
