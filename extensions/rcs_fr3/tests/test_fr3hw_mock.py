"""Tests of the FR3HW wrapper against a mocked robot, no hardware is contacted."""

from types import SimpleNamespace
from typing import Any

import gymnasium as gym
import numpy as np
import pytest
from rcs._core.common import RobotPlatform

core = pytest.importorskip("rcs_fr3._core")
envs = pytest.importorskip("rcs_fr3.envs")
hw = core.hw


class FakeFranka:
    """Stands in for hw.Franka: the control loop is simulated by pushing into a real ControlStateBuffer."""

    def __init__(self, record: bool = True):
        self.cfg = SimpleNamespace(record_control_states=record, robot_type=SimpleNamespace(id="FR3"))
        self.buffer = hw.ControlStateBuffer()
        self.fail_next_step = False
        self.recovered = 0

    def get_config(self):
        return self.cfg

    def get_state(self):
        return SimpleNamespace(robot_state=hw.RobotState())

    def drain_control_states(self):
        return self.buffer.drain()

    def clear_control_states(self):
        self.buffer.clear()

    def automatic_error_recovery(self):
        self.recovered += 1

    def stop_control_thread(self):
        pass

    def control_loop(self, n: int):
        state = hw.RobotState()
        for _ in range(n):
            self.buffer.push(state)


class FakeHardwareEnv(gym.Env):
    PLATFORM = RobotPlatform.HARDWARE

    def __init__(self, robot: FakeFranka):
        self.robot = robot

    def get_robot_obs(self) -> dict[str, Any]:
        return {"joints": np.zeros(7)}

    def step(self, action):
        if self.robot.fail_next_step:
            self.robot.fail_next_step = False
            msg = "libfranka: Move command aborted: motion aborted by reflex! ['cartesian_reflex']"
            raise hw.exceptions.FrankaControlException(msg)
        # 15 Hz env step with a 1 kHz control loop
        self.robot.control_loop(66)
        return self.get_robot_obs(), 0.0, False, False, {"platform": "hardware"}

    def reset(self, *, seed=None, options=None):
        # samples during the reset (e.g. homing) must not leak into the next episode
        self.robot.control_loop(10)
        return self.get_robot_obs(), {}


@pytest.fixture()
def _patched_hw(monkeypatch):
    fake_hw = SimpleNamespace(
        Franka=FakeFranka, FrankaState=SimpleNamespace, RobotState=hw.RobotState, exceptions=hw.exceptions
    )
    monkeypatch.setattr(envs, "hw", fake_hw)


@pytest.mark.usefixtures("_patched_hw")
def test_control_states_in_info():
    robot = FakeFranka(record=True)
    env = envs.FR3HW(FakeHardwareEnv(robot))
    env.reset()
    _, _, _, _, info = env.step({})
    states = info["control_states"]
    assert states["num_samples"] == 66
    assert states["q"].shape == (66, 7)
    assert states["tau_ext_hat_filtered"].shape == (66, 7)


@pytest.mark.usefixtures("_patched_hw")
def test_control_states_opt_in():
    robot = FakeFranka(record=False)
    env = envs.FR3HW(FakeHardwareEnv(robot))
    env.reset()
    _, _, _, _, info = env.step({})
    assert "control_states" not in info
