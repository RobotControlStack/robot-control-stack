"""FR3HW.recover() without a robot: it may only call what the compiled hw.Franka binding provides."""

from unittest import mock

from rcs_fr3._core import hw
from rcs_fr3.envs import FR3HW


def test_recover_uses_only_bound_methods():
    robot = mock.create_autospec(hw.Franka, instance=True)  # attribute access not in the binding raises
    env = FR3HW.__new__(FR3HW)
    env.hw_robot = robot
    env.recover()
    robot.automatic_error_recovery.assert_called_once_with()
