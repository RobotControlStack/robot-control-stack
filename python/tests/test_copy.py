"""The bound value types (configs, states, ...) can be copied and deep-copied,
also as Python subclasses."""

import copy

import numpy as np
import pytest
from test_sim_torque_control import Setup

import rcs
from rcs import common, sim


def joint_setpoint() -> common.JointSetpoint:
    interpolator = common.LinearJointInterpolator(0.05)
    interpolator.reset(np.zeros(7))
    return interpolator.step(0.001)


def cartesian_setpoint() -> common.CartesianSetpoint:
    interpolator = common.LinearCartesianInterpolator(0.05)
    interpolator.reset(common.Pose())
    return interpolator.step(0.001)


VALUES = {
    "BaseCameraConfig": lambda: common.BaseCameraConfig("cam", 30, 640, 480),
    "RotVec": lambda: common.RotVec(np.array([0.1, 0.2, 0.3])),
    "Pose": lambda: common.Pose(translation=np.array([0.1, 0.2, 0.3])),
    "RPY": lambda: common.RPY(0.1, 0.2, 0.3),
    "JointSetpoint": lambda: joint_setpoint(),
    "CartesianSetpoint": lambda: cartesian_setpoint(),
    "ControllerState": common.ControllerState,
    "Payload": lambda: common.Payload(mass=0.9, com=np.zeros(3), inertia=np.eye(3)),
    "EndEffectorConfig": common.EndEffectorConfig,
    "OSCConfig": common.OSCConfig,
    "JointPDConfig": lambda: common.JointPDConfig(kp=np.ones(7), kd=np.ones(7)),
    "FR3JointControllerConfig": common.FR3JointControllerConfig,
    "FR3OSCConfig": common.FR3OSCConfig,
    "SafetyConfig": lambda: common.SafetyConfig(torque_limit=np.ones(7)),
    "SafetyStage": lambda: common.SafetyStage(common.SafetyConfig(), np.zeros((2, 7)), 0.001),
    "LinearJointInterpolator": lambda: common.LinearJointInterpolator(0.05),
    "LinearCartesianInterpolator": lambda: common.LinearCartesianInterpolator(0.05),
    "RobotType": lambda: common.RobotType.FR3,
    "GripperType": lambda: common.GripperType.FrankaHand,
    "RobotConfig": lambda: rcs.ROBOTS[common.RobotType.FR3].apply(common.RobotConfig()),
    "RobotState": common.RobotState,
    "GripperConfig": common.GripperConfig,
    "GripperState": common.GripperState,
    "HandConfig": common.HandConfig,
    "HandState": common.HandState,
    "SimRobotConfig": lambda: rcs.ROBOTS[common.RobotType.FR3].apply(sim.SimRobotConfig()),
    "SimGripperConfig": sim.SimGripperConfig,
    "SimConfig": lambda: sim.SimConfig(frequency=20),
    "SimTilburgHandConfig": sim.SimTilburgHandConfig,
}


@pytest.mark.parametrize("name", list(VALUES))
@pytest.mark.parametrize("copier", [copy.copy, copy.deepcopy])
def test_value_types_can_be_copied(name, copier):
    value = VALUES[name]()
    copied = copier(value)
    assert type(copied) is type(value)
    assert copied is not value


def test_a_copy_is_independent():
    cfg = rcs.ROBOTS[common.RobotType.FR3].apply(sim.SimRobotConfig())
    cfg.controller = common.FR3OSCConfig()
    copied = copy.deepcopy(cfg)
    copied.joint_limits = np.zeros((2, 7))
    copied.end_effector.tcp_offset = common.Pose(translation=np.array([0.0, 0.0, 0.2]))
    np.testing.assert_allclose(cfg.joint_limits, rcs.ROBOTS[common.RobotType.FR3].joint_limits)
    np.testing.assert_allclose(cfg.end_effector.tcp_offset.translation(), 0.0)
    assert type(copied.controller) is common.FR3OSCConfig


def test_sim_states_can_be_copied():
    setup = Setup()
    state = setup.robot.get_state()
    assert type(copy.deepcopy(state)) is type(state)


def test_a_python_subclass_keeps_its_type_and_attributes():
    class ToolConfig(sim.SimRobotConfig):
        def __init__(self):
            super().__init__()
            rcs.ROBOTS[common.RobotType.FR3].apply(self)
            self.note = ["camera on the wrist"]

    cfg = ToolConfig()
    deep = copy.deepcopy(cfg)
    assert type(deep) is ToolConfig
    assert deep.dof == 7
    assert deep.note == cfg.note
    assert deep.note is not cfg.note
    assert copy.copy(cfg).note is cfg.note


def test_fr3_config_can_be_copied():
    hw = pytest.importorskip("rcs_fr3.hw")
    cfg = hw.FR3Config(ip="192.168.1.11", async_control=True)
    copied = copy.deepcopy(cfg)
    assert type(copied) is hw.FR3Config
    assert copied.ip == "192.168.1.11"
    assert copied.async_control
    assert copied.safety is not None
