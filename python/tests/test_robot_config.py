"""The robot specific fields of RobotConfig come from rcs.ROBOTS, not from C++ defaults."""

import numpy as np
import pytest
from test_sim_torque_control import Setup

import rcs
from rcs import common, sim


def test_robot_config_has_no_robot_specific_defaults():
    cfg = sim.SimRobotConfig()
    assert cfg.robot_type.id == ""
    assert cfg.dof == 0
    assert cfg.joint_limits.shape == (2, 0)
    assert cfg.kinematic_model_path == ""
    assert cfg.q_home is None
    # an unset type is not registered as a robot type
    assert "" not in [t.id for t in common.RobotType.get_all()]


@pytest.mark.parametrize("robot_type", list(rcs.ROBOTS))
def test_apply_sets_the_robot_specific_fields(robot_type):
    meta = rcs.ROBOTS[robot_type]
    cfg = meta.apply(sim.SimRobotConfig())
    assert cfg.robot_type == robot_type
    assert cfg.dof == meta.dof
    np.testing.assert_array_equal(cfg.joint_limits, meta.joint_limits)
    np.testing.assert_array_equal(cfg.q_home, meta.q_home)
    assert cfg.kinematic_model_path == meta.mjcf_model_path
    assert cfg.attachment_site == meta.attachment_site


def test_sim_robot_rejects_a_config_without_robot_specific_fields():
    setup = Setup()
    with pytest.raises(ValueError, match="apply"):
        setup.robot.set_config(sim.SimRobotConfig())


def test_sim_robot_needs_a_joint_and_actuator_per_dof():
    setup = Setup()
    cfg = setup.robot.get_config()
    cfg.actuators = cfg.actuators[:-1]
    with pytest.raises(ValueError, match="per dof"):
        setup.robot.set_config(cfg)


def test_platform_is_set_by_the_config_class():
    assert sim.SimRobotConfig().robot_platform == common.RobotPlatform.SIMULATION


def test_fr3_config_takes_its_values_from_rcs_robots():
    hw = pytest.importorskip("rcs_fr3.hw")
    meta = rcs.ROBOTS[common.RobotType.FR3]
    cfg = hw.FR3Config(ip="192.168.1.11", async_control=True, policy_rate=15)
    assert cfg.robot_type == common.RobotType.FR3
    assert cfg.robot_platform == common.RobotPlatform.HARDWARE
    assert cfg.dof == meta.dof
    np.testing.assert_array_equal(cfg.joint_limits, meta.joint_limits)
    assert cfg.ip == "192.168.1.11"
    assert cfg.async_control
    assert cfg.policy_rate == 15
    with pytest.raises(TypeError, match="no field"):
        hw.FR3Config(ip="192.168.1.11", asynch_control=True)  # typo
