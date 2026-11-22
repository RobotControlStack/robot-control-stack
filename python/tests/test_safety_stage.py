"""The safety stage that limits the torques of any controller."""

import mujoco
import numpy as np
import pytest
from test_controller_config import configured
from test_sim_torque_control import KD, KP

from rcs import common

DT = 0.001
LIMITS = np.array([[-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]])  # any number of joints


def state(q=(0.0, 0.0, 0.0)) -> common.ControllerState:
    s = common.ControllerState()
    s.q = np.array(q, dtype=float)
    s.dq = np.zeros(len(q))
    s.dt = DT
    return s


def stage(**config) -> common.SafetyStage:
    result = common.SafetyStage(common.SafetyConfig(**config), LIMITS, DT)
    result.reset(state())
    return result


def test_torques_pass_unchanged_within_the_limits():
    np.testing.assert_allclose(stage().apply(np.array([5.0, -3.0, 0.5]), state()), [5.0, -3.0, 0.5])


def test_torque_is_clipped():
    np.testing.assert_allclose(
        stage(torque_limit=np.array([1.0, 2.0, 3.0])).apply(np.array([5.0, -5.0, 2.0]), state()), [1.0, -2.0, 2.0]
    )


def test_torque_rate_is_limited_per_step_of_the_timestep():
    s = stage(max_torque_rate=1000.0)  # 1 Nm per 1 ms step
    first = s.apply(np.array([5.0, -5.0, 0.5]), state())
    second = s.apply(np.array([5.0, -5.0, 0.5]), state())
    np.testing.assert_allclose(first, [1.0, -1.0, 0.5])
    np.testing.assert_allclose(second, [2.0, -2.0, 0.5])
    # the measured dt does not matter (libfranka limits per command)
    late = state()
    late.dt = 0.002
    np.testing.assert_allclose(s.apply(np.array([5.0, -5.0, 0.5]), late), [3.0, -3.0, 0.5])


def test_reset_starts_from_zero_torque():
    s = stage(max_torque_rate=1000.0)
    for _ in range(10):
        s.apply(np.full(3, 5.0), state())
    s.reset(state())
    np.testing.assert_allclose(s.apply(np.full(3, 5.0), state()), 1.0)


def test_no_torque_towards_a_close_joint_limit():
    near = state(q=(0.95, -0.95, 0.0))  # 0.05 rad from the upper and the lower limit
    tau = stage().apply(np.array([1.0, -1.0, 1.0]), near)
    np.testing.assert_allclose(tau, [0.0, 0.0, 1.0])
    # away from the limit is allowed
    np.testing.assert_allclose(stage().apply(np.array([-1.0, 1.0, 1.0]), near), [-1.0, 1.0, 1.0])


def test_torque_limit_needs_one_entry_per_joint():
    with pytest.raises(ValueError, match="one entry per joint"):
        common.SafetyStage(common.SafetyConfig(torque_limit=np.ones(2)), LIMITS, DT)


def controller_torque(setup) -> np.ndarray:
    """Actuator torque without the gravity term the sim adds (the firmware's part)."""
    model, data = setup.sim.model, setup.sim.data
    still = mujoco.MjData(model)
    still.qpos[:] = data.qpos
    mujoco.mj_forward(model, still)  # zero velocity: the bias is gravity only
    return data.actuator_force[:7] - still.qfrc_bias[:7]


def test_safe_controller_limits_the_wrapped_controller():
    setup = configured(common.JointPDConfig(kp=KP, kd=KD))
    controller = common.SafeJointController(
        common.JointPD(KP, KD, common.LinearJointInterpolator(0.05)),
        common.SafetyConfig(torque_limit=np.full(7, 2.0), max_torque_rate=1000.0),
        setup.robot.get_config().joint_limits,
        DT,
    )
    setup.robot.set_joint_controller(controller)
    setup.robot.set_joint_position(setup.robot.get_joint_position() + 0.5)  # large error
    setup.run(0.1)
    assert np.max(np.abs(controller_torque(setup))) <= 2.0 + 0.05


def test_robot_config_safety_wraps_the_configured_controller():
    setup = configured(common.JointPDConfig(kp=KP, kd=KD))
    cfg = setup.robot.get_config()
    cfg.safety = common.SafetyConfig(torque_limit=np.full(7, 2.0))
    setup.robot.set_config(cfg)
    setup.robot.set_joint_position(setup.robot.get_joint_position() + 0.5)
    setup.run(0.1)
    assert np.max(np.abs(controller_torque(setup))) <= 2.0 + 0.05
