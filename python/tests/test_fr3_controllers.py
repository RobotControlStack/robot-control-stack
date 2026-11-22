"""The FR3 controllers ported from Franka.cpp: control laws, with the FR3's
safety stage (from rcs.ROBOTS) as on the robot, and their use in the
torque-controlled sim."""

import numpy as np
import pytest
from test_sim_torque_control import Setup, pose_error

import rcs
from rcs import common

DT = 0.001
Q0 = np.array([0.0, -0.8, 0.0, -2.4, 0.0, 1.6, 0.8])  # away from the joint limits
JOINT_LIMITS = rcs.ROBOTS[common.RobotType.FR3].joint_limits
# the FR3's safety stage, as RobotConfig.safety gets it from rcs.ROBOTS
SAFETY = rcs.ROBOTS[common.RobotType.FR3].apply(common.RobotConfig()).safety


@pytest.fixture()
def dynamics() -> common.PinocchioDynamics:
    path = str(rcs.ROBOTS[common.RobotType.FR3].mjcf_model_path)
    return common.PinocchioDynamics(path, False, "attachment_site", common.Pose(translation=np.array([0, 0, 0.1])))


def joint_controller(state, safety=True) -> common.JointController:
    controller = common.FR3JointController(common.FR3JointControllerConfig(), common.LinearJointInterpolator(0.05))
    if safety:
        controller = common.SafeJointController(controller, SAFETY, JOINT_LIMITS, DT)
    controller.reset(state)
    return controller


def osc(state, safety=True, **config) -> common.CartesianController:
    cfg = common.FR3OSCConfig()
    for key, value in config.items():
        setattr(cfg, key, value)
    controller = common.FR3OSC(cfg, common.LinearCartesianInterpolator(0.05))
    if safety:
        controller = common.SafeCartesianController(controller, SAFETY, JOINT_LIMITS, DT)
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
    np.testing.assert_allclose(np.abs(tau), SAFETY.torque_limit)


def test_soft_end_stop_in_front_of_the_fr3_limits(dynamics):
    # joint 6 (lower limit 0.44 rad on the FR3, -0.02 rad on the Panda) inside the spring zone in front of its
    # zero-velocity point: holding there, the stage pushes the joint away from the limit
    d0 = SAFETY.velocity_offset[5] ** 2 / (2 * SAFETY.deceleration[5])
    q = Q0.copy()
    q[5] = JOINT_LIMITS[0, 5] + d0 + 0.5 * SAFETY.spring_range
    state = dynamics.compute(q, np.zeros(7), DT)
    controller = joint_controller(state)
    for _ in range(20):  # let the rate limit settle
        tau = controller.step(state)
    np.testing.assert_allclose(tau[5], 0.5 * SAFETY.spring_torque[5], rtol=1e-6)
    np.testing.assert_allclose(tau[[0, 1, 2, 3, 4, 6]], 0.0, atol=1e-6)


def test_fast_approach_to_an_fr3_limit_is_braked(dynamics):
    # joint 1 moving at 1 rad/s towards its upper limit, 0.1 rad away: allowed there are 80% of 0.44 rad/s
    q = Q0.copy()
    q[0] = JOINT_LIMITS[1, 0] - 0.1
    dq = np.zeros(7)
    dq[0] = 1.0
    state = dynamics.compute(q, dq, DT)
    allowed = 0.8 * (np.sqrt(2 * SAFETY.deceleration[0] * 0.1) - SAFETY.velocity_offset[0])
    stage = common.SafetyStage(SAFETY, JOINT_LIMITS, DT)
    stage.reset(state)
    for _ in range(100):  # let the rate limit (1 Nm per step) settle
        tau = stage.apply(np.zeros(7), state)
    np.testing.assert_allclose(tau[0], -SAFETY.limit_damping[0] * (1.0 - allowed), rtol=1e-6)
    # moving away from the limit is not braked
    state = dynamics.compute(q, -dq, DT)
    stage.reset(state)
    assert np.all(stage.apply(np.zeros(7), state) == 0.0)


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
            common.SafeJointController(
                common.FR3JointController(common.FR3JointControllerConfig(), common.LinearJointInterpolator(0.05)),
                SAFETY,
                JOINT_LIMITS,
                DT,
            )
        )
        goal = setup.robot.get_joint_position() + np.array([0.1, -0.1, 0.1, 0.1, -0.1, 0.1, 0.2])
        setup.robot.set_joint_position(goal)
        setup.run(2.0)
        # joint friction in the model leaves a steady-state error
        assert np.max(np.abs(setup.robot.get_joint_position() - goal)) < 0.03
    else:
        q_home = setup.robot.get_joint_position()
        setup.robot.set_cartesian_controller(
            common.SafeCartesianController(
                common.FR3OSC(common.FR3OSCConfig(), common.LinearCartesianInterpolator(0.05)),
                SAFETY,
                JOINT_LIMITS,
                DT,
            )
        )
        start = setup.robot.get_cartesian_position()
        goal = common.Pose(rotation=start.rotation_m(), translation=start.translation() + np.array([0.05, -0.05, 0.0]))
        setup.robot.set_cartesian_position(goal)
        setup.run(3.0)
        position_error, _ = pose_error(setup.robot.get_cartesian_position(), goal)
        assert position_error < 0.02
        assert np.max(np.abs(setup.robot.get_joint_position() - q_home)) < 1.0  # no runaway in the null space
