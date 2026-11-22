"""Selecting a controller through RobotConfig.controller."""

import copy

import numpy as np
import pytest
from rcs.envs.base import ControlMode, JointsDictType, RelativeTo, TQuatDictType
from rcs.envs.configs import EmptyWorldFR3, EmptyWorldFR3Duo
from test_sim_torque_control import KD, KP, Setup, pose_error

from rcs import common, sim

FREQUENCY = 20  # Hz, the env's control rate in async mode


def env_config(scene, control_mode: ControlMode, controller):
    cfg = copy.deepcopy(scene.config())
    cfg.control_mode = control_mode
    cfg.task_cfg = None
    cfg.gripper_cfgs = None
    cfg.gripper_offsets = None
    cfg.camera_cfgs = None
    cfg.camera_adds = None
    cfg.headless = True
    cfg.sim_cfg.realtime = False
    cfg.sim_cfg.async_control = True
    cfg.sim_cfg.frequency = FREQUENCY
    cfg.max_relative_movement = None
    cfg.relative_to = RelativeTo.NONE
    for robot_cfg in cfg.robot_cfgs.values():
        robot_cfg.controller = controller
    return cfg


def assert_torque_controlled(env):
    # position servos became motors: no bias term (the scenes have no other actuators)
    model = env.get_wrapper_attr("sim").model
    assert model.nu > 0
    assert np.all(model.actuator_biastype == 0)


@pytest.mark.parametrize(
    "config",
    [common.JointPDConfig(kp=KP, kd=KD), common.OSCConfig(), common.FR3JointControllerConfig(), common.FR3OSCConfig()],
)
def test_robot_config_holds_any_controller_config(config):
    robot_cfg = sim.SimRobotConfig()
    assert robot_cfg.controller is None
    robot_cfg.controller = config
    assert type(robot_cfg.controller) is type(config)
    assert type(copy.deepcopy(robot_cfg).controller) is type(config)


def test_joint_controller_in_both_arms_of_a_duo_env():
    scene = EmptyWorldFR3Duo()
    env = scene.create_env(env_config(scene, ControlMode.JOINTS, common.FR3JointControllerConfig()))
    assert_torque_controlled(env)
    obs, _ = env.reset()
    offset = np.array([0.1, -0.1, 0.1, 0.1, -0.1, 0.1, 0.2])
    goals = {name: obs[name]["joints"] + offset for name in obs}
    for _ in range(2 * FREQUENCY):  # 2 s
        obs, *_ = env.step({name: JointsDictType(joints=goal) for name, goal in goals.items()})
    for name, goal in goals.items():
        # joint friction in the model leaves a steady-state error
        assert np.max(np.abs(obs[name]["joints"] - goal)) < 0.03
    env.close()


def test_cartesian_controller_in_an_env():
    scene = EmptyWorldFR3()
    env = scene.create_env(env_config(scene, ControlMode.CARTESIAN_TQuat, common.FR3OSCConfig()))
    assert_torque_controlled(env)
    obs, _ = env.reset()
    name = scene.lead_robot_name(scene.config())
    goal = obs[name]["tquat"].copy()
    goal[:3] += np.array([0.05, -0.05, 0.0])
    for _ in range(3 * FREQUENCY):  # 3 s
        obs, *_ = env.step({name: TQuatDictType(tquat=goal)})
    assert np.linalg.norm(obs[name]["tquat"][:3] - goal[:3]) < 0.02
    env.close()


def configured(controller, async_control=True) -> Setup:
    setup = Setup(async_control=async_control)
    cfg = setup.robot.get_config()
    cfg.controller = controller
    setup.robot.set_config(cfg)
    return setup


def test_joint_controller_reaches_cartesian_goals_through_ik():
    setup = configured(common.JointPDConfig(kp=KP, kd=KD))
    start = setup.robot.get_cartesian_position()
    goal = common.Pose(rotation=start.rotation_m(), translation=start.translation() + np.array([0.03, 0.0, -0.03]))
    setup.robot.set_cartesian_position(goal)
    setup.run(2.0)
    position_error, _ = pose_error(setup.robot.get_cartesian_position(), goal)
    assert position_error < 0.01


def test_configured_controller_drives_motors():
    setup = configured(common.OSCConfig())
    model = setup.sim.model
    # position servos became motors: no bias term
    assert np.all(model.actuator_biastype[:7] == 0)


def test_configured_controller_needs_async_control():
    with pytest.raises(RuntimeError, match="async_control"):
        configured(common.OSCConfig(), async_control=False)


def test_no_way_back_to_native_control():
    setup = configured(common.OSCConfig())
    cfg = setup.robot.get_config()
    cfg.controller = None
    with pytest.raises(RuntimeError):
        setup.robot.set_config(cfg)


def test_joint_pd_config_needs_gains():
    with pytest.raises(ValueError, match="kp and kd"):
        configured(common.JointPDConfig(kp=np.array([]), kd=np.array([])))


# motor inertia of the FR3 as the armature of its MJCF
FR3_ARMATURE = np.array([0.195, 0.195, 0.195, 0.195, 0.074, 0.074, 0.074])


@pytest.mark.parametrize("residual_mass", [None, FR3_ARMATURE])
def test_osc_rotates_about_the_tool_axis_only_with_residual_mass(residual_mass):
    # the model has no motor inertia: joint 7 weighs 0.0007 kg m^2, so without residual mass the OSC's torque for a
    # rotation about the tool axis stays below the joint friction (as seen on the real FR3); with the motor inertia
    # the rotation is done by joint 7, as it should
    cfg = common.OSCConfig()
    cfg.kp_orientation = np.full(3, 125.0)
    if residual_mass is not None:
        cfg.residual_mass = residual_mass
    setup = configured(cfg)
    q0 = np.array(setup.robot.get_joint_position())
    yaw = common.Pose(rpy_vector=np.array([0.0, 0.0, np.radians(30)]), translation=np.zeros(3))
    setup.robot.set_cartesian_position(setup.robot.get_cartesian_position() * yaw)
    setup.run(3.0)
    turned = np.degrees(np.array(setup.robot.get_joint_position()) - q0)
    if residual_mass is None:
        assert abs(turned[6]) < 5.0
    else:
        assert turned[6] > 25.0
        assert np.max(np.abs(turned[:6])) < 5.0


def test_osc_without_velocity_feedforward_is_smooth_with_jittery_goals():
    # 30 Hz goals with hand jitter: the setpoint velocity jumps with every goal, which the feed-forward turns into
    # torque kicks at the goal rate; damping towards zero velocity (as the FR3 OSC) does not
    rng = np.random.default_rng(0)
    rate_p99 = {}
    for feedforward in (True, False):
        cfg = common.OSCConfig()
        cfg.kp_orientation = np.full(3, 125.0)
        cfg.residual_mass = FR3_ARMATURE
        cfg.velocity_feedforward = feedforward
        setup = configured(cfg)
        start = setup.robot.get_cartesian_position()
        taus = []
        for _ in range(45):  # 1.5 s at 30 Hz
            goal = common.Pose(
                rotation=start.rotation_m(), translation=start.translation() + 0.001 * rng.normal(size=3)
            )
            setup.robot.set_cartesian_position(goal)
            for _ in range(33):
                setup.sim.step(1)
                taus.append(setup.sim.data.actuator_force[:7].copy())
        rate_p99[feedforward] = np.percentile(np.abs(np.diff(np.array(taus)[:, :4], axis=0)) * 1000, 99)
    assert rate_p99[False] < 0.3 * rate_p99[True]
