from typing import cast

import numpy as np
from rcs.envs.base import ArmWithGripper
from rcs.operator.interface import TeleopLoop


def _catch_loop(tcp_position: list[float]) -> TeleopLoop:
    loop = TeleopLoop.__new__(TeleopLoop)
    loop.catch_to_teleop = True
    loop._caught_to_teleop = False
    loop._catch_wait_message_shown = False
    loop.key_translation = {"right": "right"}
    loop._last_obs = {
        "right": {"tquat": np.array([*tcp_position, 0.0, 0.0, 0.0, 1.0])},
    }
    return loop


def _controller_action(position: list[float], quaternion: list[float] | None = None) -> dict[str, ArmWithGripper]:
    if quaternion is None:
        quaternion = [0.0, 0.0, 0.0, 1.0]
    return {
        "right": cast(
            ArmWithGripper,
            {
                "tquat": np.array([*position, *quaternion]),
                "gripper": np.array([1.0]),
            },
        )
    }


def _z_rotation(degrees: float) -> list[float]:
    half_angle = np.deg2rad(degrees) / 2.0
    return [0.0, 0.0, float(np.sin(half_angle)), float(np.cos(half_angle))]


def test_catch_to_teleop_waits_outside_five_centimeter_ball():
    loop = _catch_loop([0.0, 0.0, 0.0])

    assert not loop._try_catch_to_teleop(_controller_action([0.051, 0.0, 0.0]))
    assert not loop._caught_to_teleop


def test_catch_to_teleop_rejects_invalid_controller_position():
    loop = _catch_loop([0.0, 0.0, 0.0])

    assert not loop._try_catch_to_teleop(_controller_action([float("nan"), 0.0, 0.0]))
    assert not loop._caught_to_teleop


def test_catch_to_teleop_waits_outside_five_degree_rotation_error():
    loop = _catch_loop([0.0, 0.0, 0.0])

    assert not loop._try_catch_to_teleop(_controller_action([0.0, 0.0, 0.0], _z_rotation(5.1)))
    assert not loop._caught_to_teleop


def test_catch_to_teleop_latches_within_position_and_rotation_limits():
    loop = _catch_loop([0.4, -0.2, 0.6])

    assert loop._try_catch_to_teleop(_controller_action([0.43, -0.2, 0.6], _z_rotation(4.9)))
    assert loop._caught_to_teleop

    # Once caught, the controller can move outside the ball without pausing again.
    assert loop._try_catch_to_teleop(_controller_action([1.0, 1.0, 1.0]))


def test_catch_to_teleop_rearms_after_reset():
    loop = _catch_loop([0.0, 0.0, 0.0])
    loop._caught_to_teleop = True

    loop._reset_catch_state()

    assert not loop._caught_to_teleop
