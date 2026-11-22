"""SimRobot under torque control with the shared controllers (1 ms timestep)."""

import numpy as np
import pytest
from rcs.envs.configs import EmptyWorldFR3

from rcs import common, sim

TIMESTEP = 0.001
POLICY_PERIOD = 0.05
# the FR3 joint impedance gains from FrankaConfig
KP = np.array([100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0])
KD = np.array([20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0])


class Setup:
    def __init__(self, payload: common.Payload | None = None, friction: bool = True, async_control: bool = True):
        scene = EmptyWorldFR3()
        cfg = scene.prefixed_cfg(scene.config())
        name = scene.lead_robot_name(cfg)
        robot_cfg = cfg.robot_cfgs[name]
        robot_cfg.end_effector.payload = payload
        model = scene.create_model(cfg)
        if not friction:
            for joint in model.spec.joints:
                joint.frictionloss = 0.0
        self.sim = sim.Sim(model, sim.SimConfig(async_control=async_control))  # 1 ms timestep by default
        path, site = scene.kinematics_cfg(cfg)[name]
        ik = common.Pin(path, site, joint_limits=robot_cfg.joint_limits)
        self.robot = sim.SimRobot(self.sim, ik, robot_cfg)
        self.robot.set_dynamics(common.PinocchioDynamics(path, False, site))
        self.sim.step(1)

    def joint_pd(self) -> common.JointPD:
        controller = common.JointPD(KP, KD, common.LinearJointInterpolator(POLICY_PERIOD))
        self.robot.set_joint_controller(controller)
        return controller

    def osc(self) -> common.OSC:
        controller = common.OSC(common.OSCConfig(), common.LinearCartesianInterpolator(POLICY_PERIOD))
        self.robot.set_cartesian_controller(controller)
        return controller

    def run(self, seconds: float):
        self.sim.step(round(seconds / TIMESTEP))


def pose_error(a: common.Pose, b: common.Pose) -> tuple[float, float]:
    rotation = a.rotation_m().T @ b.rotation_m()
    angle = np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))
    return float(np.linalg.norm(a.translation() - b.translation())), float(angle)


def test_sim_config_sets_the_timestep():
    setup = Setup()
    assert setup.sim.model.opt.timestep == TIMESTEP
    setup.sim.set_config(sim.SimConfig(timestep=0.002))
    assert setup.sim.model.opt.timestep == 0.002
    with pytest.raises(ValueError, match="timestep"):
        setup.sim.set_config(sim.SimConfig(timestep=0.0))


def test_simulated_payload_is_the_gripper():
    setup = Setup()
    payload = setup.robot.get_simulated_payload()
    # the Franka Hand with its fingers
    assert 0.5 < payload.mass < 1.2
    assert 0.0 < payload.com[2] < 0.1  # in front of the flange
    assert np.allclose(payload.inertia, payload.inertia.T)


def test_joint_pd_holds_under_gravity():
    setup = Setup()
    q0 = setup.robot.get_joint_position()
    setup.joint_pd()
    setup.run(1.0)
    assert np.max(np.abs(setup.robot.get_joint_position() - q0)) < 1e-3


def test_joint_pd_reaches_goal():
    setup = Setup()
    setup.joint_pd()
    goal = setup.robot.get_joint_position() + np.array([0.2, -0.1, 0.1, 0.2, -0.2, 0.1, 0.3])
    setup.robot.set_joint_position(goal)
    setup.run(2.0)
    # joint friction in the model leaves a small steady-state error
    assert np.max(np.abs(setup.robot.get_joint_position() - goal)) < 0.02


@pytest.mark.parametrize(
    ("friction", "max_error_m", "max_error_deg"),
    # joint friction (frictionloss in the MJCF) stops the arm short of the goal,
    # as on the real robot. Without it the position converges exactly; the
    # orientation keeps a small offset because the rotational stiffness at the
    # wrist is low and the fingers are not rigid like the payload model.
    [(False, 1e-4, 0.5), (True, 0.015, 3.0)],
    ids=["no_friction", "friction"],
)
def test_osc_holds_and_reaches_goal(friction, max_error_m, max_error_deg):
    setup = Setup(friction=friction)
    start = setup.robot.get_cartesian_position()
    setup.osc()
    setup.run(1.0)
    held = pose_error(setup.robot.get_cartesian_position(), start)
    assert held[0] < 2e-3 and held[1] < 0.5

    goal = common.Pose(
        rotation=start.rotation_m() @ common.RPY(0.1, -0.1, 0.2).rotation_matrix(),
        translation=start.translation() + np.array([0.05, -0.08, -0.06]),
    )
    setup.robot.set_cartesian_position(goal)
    setup.run(2.0)
    reached = pose_error(setup.robot.get_cartesian_position(), goal)
    assert reached[0] < max_error_m and reached[1] < max_error_deg


def test_switching_between_controllers():
    setup = Setup(friction=False)
    setup.joint_pd()
    setup.osc()
    start = setup.robot.get_cartesian_position()
    goal = common.Pose(rotation=start.rotation_m(), translation=start.translation() + np.array([0.0, 0.05, 0.0]))
    setup.robot.set_cartesian_position(goal)
    setup.run(1.5)
    assert pose_error(setup.robot.get_cartesian_position(), goal)[0] < 1e-3

    q_goal = setup.robot.get_joint_position() + 0.1
    setup.robot.set_joint_position(q_goal)  # back to the joint controller
    setup.run(1.5)
    assert np.max(np.abs(setup.robot.get_joint_position() - q_goal)) < 1e-3


def test_wrong_payload_acts_on_the_arm():
    reference = Setup()  # keep the Sim alive, it owns the MuJoCo model
    simulated = reference.robot.get_simulated_payload()
    heavier = common.Payload(mass=simulated.mass + 1.0, com=simulated.com, inertia=simulated.inertia)
    errors = {}
    for name, payload in [("correct", None), ("wrong", heavier)]:
        setup = Setup(payload)
        q0 = setup.robot.get_joint_position()
        setup.joint_pd()
        setup.run(1.0)
        errors[name] = np.max(np.abs(setup.robot.get_joint_position() - q0))
    # 1 kg too much compensation lifts the arm, as a wrong Desk setting would
    assert errors["wrong"] > 10 * errors["correct"]


def test_set_joints_hard_holds_the_new_configuration():
    setup = Setup()
    setup.joint_pd()
    q = setup.robot.get_joint_position() + 0.2
    setup.robot.set_joints_hard(q)
    setup.run(0.5)
    assert np.max(np.abs(setup.robot.get_joint_position() - q)) < 1e-2


def test_controller_needs_dynamics():
    scene = EmptyWorldFR3()
    cfg = scene.prefixed_cfg(scene.config())
    name = scene.lead_robot_name(cfg)
    simulation = sim.Sim(scene.create_model(cfg), sim.SimConfig())
    path, site = scene.kinematics_cfg(cfg)[name]
    robot = sim.SimRobot(simulation, common.Pin(path, site), cfg.robot_cfgs[name])
    with pytest.raises(RuntimeError):
        robot.set_joint_controller(common.JointPD(KP, KD, common.LinearJointInterpolator(POLICY_PERIOD)))
