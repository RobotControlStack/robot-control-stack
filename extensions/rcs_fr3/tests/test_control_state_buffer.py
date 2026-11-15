"""Tests of the control state ring buffer, they do not need a robot."""

import numpy as np
import pytest

hw = pytest.importorskip("rcs_fr3._core").hw


def test_drain_shapes_and_order():
    buffer = hw.ControlStateBuffer(100)
    assert buffer.capacity() == 128
    state = hw.RobotState()
    for _ in range(66):
        assert buffer.push(state)
    assert buffer.size() == 66
    states = buffer.drain()
    assert states["num_samples"] == 66
    assert states["dropped"] == 0
    assert states["timestamp"].shape == (66,)
    assert states["q"].shape == (66, 7)
    assert states["tau_J"].shape == (66, 7)
    assert states["dtau_J"].shape == (66, 7)
    assert states["tau_ext_hat_filtered"].shape == (66, 7)
    assert states["O_F_ext_hat_K"].shape == (66, 6)
    assert states["O_T_EE"].shape == (66, 16)
    assert states["robot_mode"].dtype == np.int32
    assert states["current_errors"].dtype == np.uint64
    # timestamps are taken at push time and must be monotonic
    assert np.all(np.diff(states["timestamp"]) >= 0)
    assert buffer.size() == 0
    assert buffer.drain()["num_samples"] == 0


def test_overflow_drops_newest_and_counts():
    buffer = hw.ControlStateBuffer(8)
    state = hw.RobotState()
    pushed = [buffer.push(state) for _ in range(10)]
    assert pushed == [True] * 8 + [False] * 2
    states = buffer.drain()
    assert states["num_samples"] == 8
    assert states["dropped"] == 2
    # counter is reset after it was reported
    assert buffer.drain()["dropped"] == 0


def test_error_names():
    names = hw.control_state_error_names()
    assert len(names) == 41
    assert names[0] == "joint_position_limits_violation"
    assert "joint_reflex" in names
    assert "cartesian_reflex" in names


def test_config_flag_is_opt_in():
    cfg = hw.FR3Config(ip="localhost")
    assert cfg.record_control_states is False
    cfg.record_control_states = True
    assert cfg.record_control_states is True
