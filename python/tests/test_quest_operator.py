import threading
from typing import Any, cast

import numpy as np
from rcs._core.common import Pose
from rcs.operator.interface import TeleopCommands
from rcs.operator.quest import QuestOperator


class _Config:
    switched_left_right = True
    include_rotation = True
    absolute_tracking = False


def test_swap_controller_input_swaps_left_and_right_packets():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())

    input_data = {
        "left": {"hand_trigger": 0.1, "index_trigger": 0.2},
        "right": {"hand_trigger": 0.9, "index_trigger": 0.8},
        "A": True,
    }

    swapped = QuestOperator._swap_controller_input(operator, input_data)

    assert swapped["left"] == input_data["right"]
    assert swapped["right"] == input_data["left"]
    assert swapped["A"] is True


def test_consume_commands_swaps_both_reset_origin_keys():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())
    operator._cmd_lock = threading.Lock()
    operator._commands = TeleopCommands(reset_origin_to_current={"right": True, "left": False})

    cmds = QuestOperator.consume_commands(operator)

    assert cmds.reset_origin_to_current == {"right": True, "left": False}
    assert operator._commands.reset_origin_to_current == {}


def test_consume_action_swaps_gripper_with_controller():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())
    operator._resource_lock = threading.Lock()
    operator.controller_names = ["left", "right"]
    operator._last_controller_pose = {key: Pose() for key in operator.controller_names}
    operator._offset_pose = {key: Pose() for key in operator.controller_names}
    operator._set_frame = {key: Pose() for key in operator.controller_names}
    operator._grp_pos = {"left": 0.75, "right": 0.25}

    actions = QuestOperator.consume_action(operator)

    assert np.allclose(actions["left"]["gripper"], np.array([0.75]))
    assert np.allclose(actions["right"]["gripper"], np.array([0.25]))


def test_consume_action_returns_absolute_controller_pose():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())
    operator.config.absolute_tracking = True
    operator._resource_lock = threading.Lock()
    operator.controller_names = ["right"]
    target = Pose(
        translation=np.array([0.4, -0.2, 0.6]),
        quaternion=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    operator._last_controller_pose = {"right": target}
    operator._controller_pose_valid = {"right": True}
    operator._offset_pose = {"right": Pose(translation=np.array([10.0, 10.0, 10.0]))}
    operator._set_frame = {"right": Pose()}
    operator._grp_pos = {"right": 0.5}

    action = QuestOperator.consume_action(operator)["right"]

    assert np.allclose(action["tquat"][:3], target.translation())
    assert np.allclose(action["tquat"][3:], target.rotation_q())


def test_consume_action_waits_for_first_absolute_pose():
    operator = QuestOperator.__new__(QuestOperator)
    operator.config = cast(Any, _Config())
    operator.config.absolute_tracking = True
    operator._resource_lock = threading.Lock()
    operator.controller_names = ["right"]
    operator._last_controller_pose = {"right": Pose()}
    operator._controller_pose_valid = {"right": False}
    operator._offset_pose = {"right": Pose()}
    operator._set_frame = {"right": Pose()}
    operator._grp_pos = {"right": 0.5}

    assert QuestOperator.consume_action(operator) == {}
