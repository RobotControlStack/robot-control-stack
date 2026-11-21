"""Control laws of JointPD and OSC on FR3 states from PinocchioDynamics."""

import numpy as np
import pytest

import rcs
from rcs import common

DT = 0.001
Q0 = np.array([0.0, -0.5, 0.0, -2.2, 0.0, 1.8, 0.8])


@pytest.fixture
def dynamics() -> common.PinocchioDynamics:
    path = str(rcs.ROBOTS[common.RobotType.FR3].mjcf_model_path)
    return common.PinocchioDynamics(path, False, "attachment_site", common.Pose(translation=np.array([0, 0, 0.1])))


def rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TestJointPD:
    kp = np.full(7, 100.0)
    kd = np.full(7, 20.0)

    def make(self, state) -> common.JointPD:
        controller = common.JointPD(self.kp, self.kd, common.LinearJointInterpolator(0.05))
        controller.reset(state)
        return controller

    def test_holds_without_torque_at_rest(self, dynamics):
        state = dynamics.compute(Q0, np.zeros(7), DT)
        np.testing.assert_allclose(self.make(state).step(state), 0.0, atol=1e-12)

    def test_pd_law_with_setpoint_velocity_and_coriolis(self, dynamics):
        controller = self.make(dynamics.compute(Q0, np.zeros(7), DT))
        goal = Q0 + 0.1
        controller.set_goal(goal)
        dq = np.full(7, 0.3)
        state = dynamics.compute(Q0, dq, DT)
        velocity = (goal - Q0) / 0.05
        expected = self.kp * (Q0 + velocity * DT - Q0) + self.kd * (velocity - dq) + state.coriolis
        np.testing.assert_allclose(controller.step(state), expected, atol=1e-10)

    def test_rejects_mismatched_gains(self):
        with pytest.raises(ValueError):
            common.JointPD(np.ones(7), np.ones(6), common.LinearJointInterpolator(0.05))


class TestOSC:
    def make(self, state, **config) -> common.OSC:
        cfg = common.OSCConfig()
        for key, value in config.items():
            setattr(cfg, key, value)
        controller = common.OSC(cfg, common.LinearCartesianInterpolator(0.05))
        controller.reset(state)
        return controller

    @staticmethod
    def task_acceleration(state, tau) -> np.ndarray:
        """TCP acceleration the torque causes at rest (without gravity, Coriolis)."""
        return state.jacobian @ np.linalg.solve(state.mass, tau - state.coriolis)

    def test_holds_without_torque_at_rest(self, dynamics):
        state = dynamics.compute(Q0, np.zeros(7), DT)
        np.testing.assert_allclose(self.make(state).step(state), 0.0, atol=1e-9)

    def test_position_error_gives_kp_times_error(self, dynamics):
        state = dynamics.compute(Q0, np.zeros(7), DT)
        controller = self.make(state)
        offset = np.array([0.01, -0.02, 0.005])
        goal = common.Pose(rotation=state.tcp_pose.rotation_m(), translation=state.tcp_pose.translation() + offset)
        controller.set_goal(goal, duration=DT)  # reached after one step, zero setpoint velocity
        acceleration = self.task_acceleration(state, controller.step(state))
        np.testing.assert_allclose(acceleration[:3], 150.0 * offset, atol=1e-9)

    def test_orientation_error_gives_kp_times_error(self, dynamics):
        state = dynamics.compute(Q0, np.zeros(7), DT)
        controller = self.make(state)
        goal = common.Pose(rotation=rot_z(0.1) @ state.tcp_pose.rotation_m(), translation=state.tcp_pose.translation())
        controller.set_goal(goal, duration=DT)
        acceleration = self.task_acceleration(state, controller.step(state))
        np.testing.assert_allclose(acceleration[3:], [0.0, 0.0, 250.0 * 0.1], atol=1e-9)

    def test_damps_tcp_velocity(self, dynamics):
        rest = dynamics.compute(Q0, np.zeros(7), DT)
        controller = self.make(rest)
        # purely linear TCP motion
        velocity = np.array([0.1, -0.05, 0.08, 0.0, 0.0, 0.0])
        dq = np.linalg.pinv(rest.jacobian) @ velocity
        state = dynamics.compute(Q0, dq, DT)
        acceleration = self.task_acceleration(state, controller.step(state))
        np.testing.assert_allclose(acceleration[:3], -2.0 * np.sqrt(150.0) * velocity[:3], atol=1e-9)

    def test_posture_task_does_not_move_the_tcp(self, dynamics):
        state = dynamics.compute(Q0, np.zeros(7), DT)
        # posture goal away from the current configuration, TCP already at its goal
        controller = self.make(state, nullspace_q=Q0 + np.array([0.3, 0, 0.3, 0, 0.3, 0, 0.3]), kp_nullspace=20.0)
        tau = controller.step(state)
        assert np.linalg.norm(tau) > 0.1
        np.testing.assert_allclose(self.task_acceleration(state, tau), 0.0, atol=1e-9)

    def test_step_before_reset_raises(self, dynamics):
        controller = common.OSC(common.OSCConfig(), common.LinearCartesianInterpolator(0.05))
        with pytest.raises(RuntimeError):
            controller.step(dynamics.compute(Q0, np.zeros(7), DT))
