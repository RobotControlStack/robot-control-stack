"""The FR3 controllers ported from Franka.cpp: control laws, safety stage, and
their use in the torque-controlled sim."""

import numpy as np
import pytest
from test_sim_torque_control import Setup, pose_error

import rcs
from rcs import common

DT = 0.001
Q0 = np.array([0.0, -0.8, 0.0, -2.4, 0.0, 1.6, 0.8])  # away from the joint limits


@pytest.fixture
def dynamics() -> common.PinocchioDynamics:
    path = str(rcs.ROBOTS[common.RobotType.FR3].mjcf_model_path)
    return common.PinocchioDynamics(path, False, "attachment_site", common.Pose(translation=np.array([0, 0, 0.1])))


def joint_controller(state) -> common.FR3JointController:
    controller = common.FR3JointController(common.FR3JointControllerConfig(), common.LinearJointInterpolator(0.05))
    controller.reset(state)
    return controller


def osc(state, **config) -> common.FR3OSC:
    cfg = common.FR3OSCConfig()
    for key, value in config.items():
        setattr(cfg, key, value)
    controller = common.FR3OSC(cfg, common.LinearCartesianInterpolator(0.05))
    controller.reset(state)
    return controller


def test_joint_controller_law(dynamics):
    state = dynamics.compute(Q0, np.zeros(7), DT)
    controller = joint_controller(state)
    # small error and velocity, so that the rate limit doesn't act
    q = Q0 + 0.002
    dq = np.full(7, 0.01)
    state = dynamics.compute(q, dq, DT)
    cfg = common.FR3JointControllerConfig()
    expected = cfg.kp * (Q0 - q) - cfg.kd * dq + state.coriolis  # no setpoint velocity feed-forward
    np.testing.assert_allclose(controller.step(state), expected, atol=1e-10)


def test_torque_rate_is_limited_to_1000_nm_per_s(dynamics):
    state = dynamics.compute(Q0, np.zeros(7), DT)
    controller = joint_controller(state)
    controller.set_goal(Q0 + 0.5, duration=DT)  # large error at once
    first = controller.step(state)
    second = controller.step(state)
    assert np.all(np.abs(first) <= 1.0)
    assert np.all(np.abs(second - first) <= 1.0)
    assert np.max(np.abs(second)) > 1.5


def test_torque_is_clipped(dynamics):
    state = dynamics.compute(Q0, np.zeros(7), DT)
    controller = joint_controller(state)
    controller.set_goal(Q0 + 1.0, duration=DT)
    for _ in range(200):
        tau = controller.step(state)
    np.testing.assert_allclose(np.abs(tau), common.FR3TorqueLimits().torque_limit)


def test_no_torque_towards_a_close_joint_limit(dynamics):
    limits = common.FR3TorqueLimits()
    q = Q0.copy()
    q[0] = limits.joint_max[0] - 0.05
    state = dynamics.compute(q, np.zeros(7), DT)
    controller = joint_controller(state)
    controller.set_goal(q + 0.3, duration=DT)  # pulls joint 1 into the limit
    for _ in range(20):
        tau = controller.step(state)
    assert tau[0] == 0.0
    assert tau[1] > 0.0


def test_osc_holds_without_torque_at_its_posture(dynamics):
    state = dynamics.compute(Q0, np.zeros(7), DT)
    np.testing.assert_allclose(osc(state, nullspace_q=Q0).step(state), 0.0, atol=1e-9)


def test_osc_pulls_towards_the_goal(dynamics):
    state = dynamics.compute(Q0, np.zeros(7), DT)
    controller = osc(state, nullspace_q=Q0)
    offset = np.array([0.02, 0.0, -0.01])
    goal = common.Pose(rotation=state.tcp_pose.rotation_m(), translation=state.tcp_pose.translation() + offset)
    controller.set_goal(goal, duration=DT)
    for _ in range(20):  # let the rate limit settle
        tau = controller.step(state)
    # the controller's own mass matrix includes the residual mass on joints 5-7
    mass = state.mass + np.diag(common.FR3OSCConfig().residual_mass)
    acceleration = state.jacobian[:3] @ np.linalg.solve(mass, tau - state.coriolis)
    np.testing.assert_allclose(acceleration, 150.0 * offset, atol=1e-6)


@pytest.mark.parametrize("controller", ["joint", "osc"])
def test_fr3_controllers_in_sim(controller):
    setup = Setup()
    if controller == "joint":
        setup.robot.set_joint_controller(
            common.FR3JointController(common.FR3JointControllerConfig(), common.LinearJointInterpolator(0.05))
        )
        goal = setup.robot.get_joint_position() + np.array([0.1, -0.1, 0.1, 0.1, -0.1, 0.1, 0.2])
        setup.robot.set_joint_position(goal)
        setup.run(2.0)
        # joint friction in the model leaves a steady-state error
        assert np.max(np.abs(setup.robot.get_joint_position() - goal)) < 0.03
    else:
        q_home = setup.robot.get_joint_position()
        setup.robot.set_cartesian_controller(
            common.FR3OSC(common.FR3OSCConfig(), common.LinearCartesianInterpolator(0.05))
        )
        start = setup.robot.get_cartesian_position()
        goal = common.Pose(rotation=start.rotation_m(), translation=start.translation() + np.array([0.05, -0.05, 0.0]))
        setup.robot.set_cartesian_position(goal)
        setup.run(3.0)
        position_error, _ = pose_error(setup.robot.get_cartesian_position(), goal)
        assert position_error < 0.02
        assert np.max(np.abs(setup.robot.get_joint_position() - q_home)) < 1.0  # no runaway in the null space
