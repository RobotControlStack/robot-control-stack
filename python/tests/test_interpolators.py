import numpy as np
import pytest

from rcs import common

DURATION = 0.05  # one policy period at 20 Hz
DT = 0.001  # 1 kHz servo rate
STEPS = round(DURATION / DT)


def rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def run(interpolator, steps: int):
    setpoint = None
    for _ in range(steps):
        setpoint = interpolator.step(DT)
    return setpoint


class TestLinearJointInterpolator:
    start = np.zeros(7)
    goal = np.linspace(0.1, 0.7, 7)

    def make(self) -> common.LinearJointInterpolator:
        interpolator = common.LinearJointInterpolator(DURATION)
        interpolator.reset(self.start)
        return interpolator

    def test_reset_holds(self):
        setpoint = run(self.make(), 10)
        assert np.allclose(setpoint.q, self.start)
        assert np.allclose(setpoint.dq, 0.0)

    def test_moves_linearly_with_constant_velocity(self):
        interpolator = self.make()
        interpolator.set_goal(self.goal)
        velocity = (self.goal - self.start) / DURATION
        half = run(interpolator, STEPS // 2)
        assert np.allclose(half.q, (self.start + self.goal) / 2)
        assert np.allclose(half.dq, velocity)

    def test_reaches_goal_after_duration_and_holds(self):
        interpolator = self.make()
        interpolator.set_goal(self.goal)
        assert np.allclose(run(interpolator, STEPS).q, self.goal)
        held = run(interpolator, 10)
        assert np.allclose(held.q, self.goal)
        assert np.allclose(held.dq, 0.0)

    def test_new_goal_starts_at_current_setpoint(self):
        interpolator = self.make()
        interpolator.set_goal(self.goal)
        before = run(interpolator, STEPS // 2).q
        # back to the start before the first segment is finished
        interpolator.set_goal(self.start)
        after = interpolator.step(DT)
        velocity = (self.start - before) / DURATION
        assert np.allclose(after.q, before + velocity * DT)
        assert np.allclose(after.dq, velocity)
        assert np.allclose(run(interpolator, STEPS).q, self.start)

    def test_duration_override_for_one_goal(self):
        interpolator = self.make()
        interpolator.set_goal(self.goal, duration=4 * DURATION)
        assert np.allclose(run(interpolator, 2 * STEPS).q, (self.start + self.goal) / 2)
        assert np.allclose(run(interpolator, 2 * STEPS).q, self.goal)
        # the next goal uses the default duration again
        interpolator.set_goal(self.start)
        assert np.allclose(run(interpolator, STEPS).q, self.start)

    def test_invalid_use_raises(self):
        with pytest.raises(ValueError, match="duration"):
            common.LinearJointInterpolator(0.0)
        with pytest.raises(RuntimeError):
            common.LinearJointInterpolator(DURATION).set_goal(self.goal)
        with pytest.raises(ValueError, match="number of joints"):
            self.make().set_goal(np.zeros(6))


class TestLinearCartesianInterpolator:
    start = common.Pose(rotation=rot_z(0.0), translation=np.array([0.3, 0.0, 0.5]))
    goal = common.Pose(rotation=rot_z(0.4), translation=np.array([0.4, -0.1, 0.4]))

    def make(self) -> common.LinearCartesianInterpolator:
        interpolator = common.LinearCartesianInterpolator(DURATION)
        interpolator.reset(self.start)
        return interpolator

    def test_reset_holds(self):
        setpoint = run(self.make(), 10)
        assert setpoint.pose.is_close(self.start)
        assert np.allclose(setpoint.twist, 0.0)

    def test_half_way_and_twist(self):
        interpolator = self.make()
        interpolator.set_goal(self.goal)
        half = run(interpolator, STEPS // 2)
        expected = common.Pose(rotation=rot_z(0.2), translation=np.array([0.35, -0.05, 0.45]))
        assert half.pose.is_close(expected, eps_r=1e-9, eps_t=1e-9)
        linear = (self.goal.translation() - self.start.translation()) / DURATION
        angular = np.array([0.0, 0.0, 0.4]) / DURATION
        assert np.allclose(half.twist, np.concatenate([linear, angular]))

    def test_reaches_goal_after_duration_and_holds(self):
        interpolator = self.make()
        interpolator.set_goal(self.goal)
        assert run(interpolator, STEPS).pose.is_close(self.goal)
        held = run(interpolator, 10)
        assert held.pose.is_close(self.goal)
        assert np.allclose(held.twist, 0.0)

    def test_rotates_the_short_way(self):
        interpolator = self.make()
        # 350 degrees about z is -10 degrees the short way round
        goal = common.Pose(rotation=rot_z(np.deg2rad(350)), translation=self.start.translation())
        interpolator.set_goal(goal)
        half = run(interpolator, STEPS // 2)
        assert half.pose.is_close(common.Pose(rotation=rot_z(np.deg2rad(-5)), translation=self.start.translation()))
        assert np.allclose(half.twist[3:], [0.0, 0.0, np.deg2rad(-10) / DURATION])

    def test_new_goal_starts_at_current_setpoint(self):
        interpolator = self.make()
        interpolator.set_goal(self.goal)
        before = run(interpolator, STEPS // 2).pose
        interpolator.set_goal(self.start)
        after = interpolator.step(DT).pose
        # one servo tick moves only a small step away from the current setpoint
        assert np.linalg.norm(after.translation() - before.translation()) < 1e-2
        assert run(interpolator, STEPS).pose.is_close(self.start)

    def test_invalid_use_raises(self):
        with pytest.raises(ValueError, match="duration"):
            common.LinearCartesianInterpolator(-1.0)
        with pytest.raises(RuntimeError):
            common.LinearCartesianInterpolator(DURATION).step(DT)
