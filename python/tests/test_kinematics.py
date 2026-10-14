import mujoco
import numpy as np
import pytest

import rcs
from rcs import common

# Restrict this test to robot models that are expected to work through the generic Pin binding.
# Pinocchio drops the Panda attachment site as it sits on a body without a joint.
PIN_SUPPORTED_ROBOTS = [
    common.RobotType.FR3,
    common.RobotType("XArm7"),
    common.RobotType("UR5e"),
    common.RobotType("SO101"),
    common.RobotType("Yam"),
]

# Robots with a genuine redundant DOF (7-DoF arms), where null-space biasing has room to act.
REDUNDANT_ROBOTS = [
    common.RobotType.FR3,
    common.RobotType("XArm7"),
]

# Identity pose / no TCP offset, reused across tests.
NO_TCP_OFFSET = common.Pose()


@pytest.mark.parametrize("robot_name", PIN_SUPPORTED_ROBOTS)
def test_kinematics_identity(robot_name):
    robot = rcs.ROBOTS[robot_name]
    model_path = robot.mjcf_model_path
    frame_id = robot.attachment_site
    q_home = robot.q_home

    # Default Pin: limit-clamping on, no null-space bias.
    try:
        pin = common.Pin(model_path, frame_id, False)
    except Exception as e:
        pytest.fail(f"Failed to initialize Pin for {robot_name}: {e}")

    # Test 1: FK at home.
    pose_home = pin.forward(q_home, NO_TCP_OFFSET)
    assert isinstance(pose_home, common.Pose)

    # Test 2: IK at the home pose returns a solution reaching that pose. The home
    # configuration is within the joint limits, so clamping does not interfere.
    q_sol: np.ndarray | None = pin.inverse(pose_home, q_home, NO_TCP_OFFSET)
    assert q_sol is not None, "IK failed for home pose"

    pose_sol = pin.forward(q_sol, NO_TCP_OFFSET)
    assert pose_sol.is_close(
        pose_home, eps_r=1e-4, eps_t=1e-4
    ), f"FK(IK(pose)) does not match pose.\nOriginal: {pose_home}\nResult: {pose_sol}"

    # Test 3: Perturbed configuration. We disable limit clamping here so that
    # reachability of FK(q_perturbed) does not depend on how close q_home sits to
    # a joint limit (e.g. SO101), keeping this a pure IK-convergence check.
    pin_free = common.Pin(model_path, frame_id, False, np.array([]), 0.0, False)

    np.random.seed(42)
    q_perturbed = q_home + np.random.uniform(-0.1, 0.1, size=q_home.shape)

    pose_perturbed = pin_free.forward(q_perturbed, NO_TCP_OFFSET)
    q_sol_perturbed: np.ndarray | None = pin_free.inverse(pose_perturbed, q_home, NO_TCP_OFFSET)
    assert q_sol_perturbed is not None, "IK failed for perturbed pose"

    pose_sol_perturbed = pin_free.forward(q_sol_perturbed, NO_TCP_OFFSET)
    assert pose_sol_perturbed.is_close(
        pose_perturbed, eps_r=1e-3, eps_t=1e-3
    ), f"FK(IK(perturbed_pose)) does not match.\nOriginal: {pose_perturbed}\nResult: {pose_sol_perturbed}"


@pytest.mark.parametrize("robot_name", REDUNDANT_ROBOTS)
def test_kinematics_nullspace_bias(robot_name):
    """A null-space target biases the redundant DOF toward the preferred posture
    without changing the achieved end-effector pose."""
    robot = rcs.ROBOTS[robot_name]
    model_path = robot.mjcf_model_path
    frame_id = robot.attachment_site
    q_home = robot.q_home

    # Clamping off on both so the comparison isolates the null-space term.
    pin_plain = common.Pin(model_path, frame_id, False, np.array([]), 0.0, False)
    pin_ns = common.Pin(model_path, frame_id, False, q_home, 2.0, False)  # bias toward home

    # Target the home pose; a seed away from home exercises the redundancy so the
    # two solvers can settle on different configurations for the same pose.
    pose_home = pin_plain.forward(q_home, NO_TCP_OFFSET)
    q_seed = q_home.copy()
    q_seed[0] += 0.5
    q_seed[2] += 0.5
    q_seed[3] += 0.3

    q_plain: np.ndarray | None = pin_plain.inverse(pose_home, q_seed, NO_TCP_OFFSET)
    q_ns: np.ndarray | None = pin_ns.inverse(pose_home, q_seed, NO_TCP_OFFSET)

    assert q_plain is not None, "plain IK failed"
    assert q_ns is not None, "null-space IK failed"

    # Both solutions must reach the same end-effector pose.
    assert pin_plain.forward(q_plain, NO_TCP_OFFSET).is_close(pose_home, eps_r=1e-3, eps_t=1e-3)
    assert pin_ns.forward(q_ns, NO_TCP_OFFSET).is_close(pose_home, eps_r=1e-3, eps_t=1e-3)

    # The null-space-biased solution sits closer to the preferred (home) posture.
    d_plain = float(np.linalg.norm(q_plain - q_home))
    d_ns = float(np.linalg.norm(q_ns - q_home))
    assert d_ns < d_plain, f"null-space bias did not pull toward home: d_ns={d_ns} vs d_plain={d_plain}"


@pytest.mark.parametrize("gain", [1.0, 10.0, 100.0])
def test_kinematics_nullspace_large_gain_converges(gain):
    robot = rcs.ROBOTS[common.RobotType.FR3]
    pin = common.Pin(robot.mjcf_model_path, robot.attachment_site, False, robot.q_home, gain)
    pose_home = pin.forward(robot.q_home, NO_TCP_OFFSET)
    q_seed = robot.q_home + np.array([0.5, 0.0, 0.5, 0.3, 0.0, 0.0, 0.0])

    q_sol = pin.inverse(pose_home, q_seed, NO_TCP_OFFSET)

    assert q_sol is not None, f"IK failed with null-space gain {gain}"
    assert pin.forward(q_sol, NO_TCP_OFFSET).is_close(pose_home, eps_r=1e-3, eps_t=1e-3)


@pytest.mark.parametrize("robot_name", REDUNDANT_ROBOTS)
def test_kinematics_nullspace_applies_when_seed_reaches_pose(robot_name):
    """A seed that already reaches the target still moves toward the posture."""
    robot = rcs.ROBOTS[robot_name]
    pin_plain = common.Pin(robot.mjcf_model_path, robot.attachment_site, False, np.array([]), 0.0, False)
    pin_ns = common.Pin(robot.mjcf_model_path, robot.attachment_site, False, robot.q_home, 10.0, False)
    pose_home = pin_plain.forward(robot.q_home, NO_TCP_OFFSET)
    q_seed = robot.q_home.copy()
    q_seed[0] += 0.5
    q_seed[2] += 0.5
    q_reached: np.ndarray | None = pin_plain.inverse(pose_home, q_seed, NO_TCP_OFFSET)
    assert q_reached is not None

    q_ns: np.ndarray | None = pin_ns.inverse(pose_home, q_reached, NO_TCP_OFFSET)

    assert q_ns is not None
    assert pin_ns.forward(q_ns, NO_TCP_OFFSET).is_close(pose_home, eps_r=1e-3, eps_t=1e-3)
    assert np.linalg.norm(q_ns - robot.q_home) < np.linalg.norm(q_reached - robot.q_home)


@pytest.mark.parametrize("robot_name", REDUNDANT_ROBOTS)
def test_kinematics_nullspace_never_loses_solutions(robot_name):
    """Targets the plain solver reaches stay reachable with the posture task."""
    robot = rcs.ROBOTS[robot_name]
    pin_plain = common.Pin(robot.mjcf_model_path, robot.attachment_site, False)
    pin_ns = common.Pin(robot.mjcf_model_path, robot.attachment_site, False, robot.q_home, 10.0)
    lower, upper = robot.joint_limits
    rng = np.random.default_rng(1)
    for _ in range(50):
        q_target = rng.uniform(lower, upper)
        q_seed = np.clip(q_target + rng.uniform(-0.3, 0.3, q_target.shape), lower, upper)
        pose = pin_plain.forward(q_target, NO_TCP_OFFSET)
        if pin_plain.inverse(pose, q_seed, NO_TCP_OFFSET) is not None:
            assert pin_ns.inverse(pose, q_seed, NO_TCP_OFFSET) is not None


def test_kinematics_missing_frame_raises():
    robot = rcs.ROBOTS[common.RobotType.FR3]
    with pytest.raises(RuntimeError, match="not_a_frame"):
        common.Pin(robot.mjcf_model_path, "not_a_frame", False)


MOUNTED_ARM_MJCF = """
<mujoco>
  <compiler angle="radian"/>
  <worldbody>
    <body name="mount" pos="1 0 0">
      <body name="robot_base" pos="0.1 0.2 0.3" quat="0.7071068 0 0 0.7071068">
        <body name="link1">
          <joint name="joint1" axis="0 0 1" range="-3 3"/>
          <geom type="capsule" size="0.02" fromto="0 0 0 0 0 0.3"/>
          <body name="link2" pos="0 0 0.3">
            <joint name="joint2" axis="0 1 0" range="-2 2"/>
            <geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0"/>
            <body name="link3" pos="0.3 0 0">
              <joint name="joint3" axis="0 1 0" range="-2 2"/>
              <geom type="capsule" size="0.02" fromto="0 0 0 0.2 0 0"/>
              <site name="tip" pos="0.2 0 0"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def test_kinematics_base_frame(tmp_path):
    """Poses are expressed in base_frame, matching MuJoCo's site pose relative to that body."""
    path = tmp_path / "mounted_arm.xml"
    path.write_text(MOUNTED_ARM_MJCF)
    pin = common.Pin(str(path), "tip", False, base_frame="robot_base")
    q = np.array([0.4, -0.3, 0.5])

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    base = data.body("robot_base")
    base_pose = common.Pose(base.xmat.reshape(3, 3), base.xpos)
    site_pose = common.Pose(data.site("tip").xmat.reshape(3, 3), data.site("tip").xpos)
    expected = base_pose.inverse() * site_pose

    pose = pin.forward(q, NO_TCP_OFFSET)
    assert pose.is_close(expected, eps_r=1e-9, eps_t=1e-9)

    tcp_offset = common.Pose(translation=np.array([0.0, 0.0, 0.1]))
    pose_tcp = pin.forward(q, tcp_offset)
    q_sol: np.ndarray | None = pin.inverse(pose_tcp, q + 0.1, tcp_offset)
    assert q_sol is not None
    assert pin.forward(q_sol, tcp_offset).is_close(pose_tcp, eps_r=1e-3, eps_t=1e-3)


def test_kinematics_base_frame_moved_by_joint_raises(tmp_path):
    path = tmp_path / "mounted_arm.xml"
    path.write_text(MOUNTED_ARM_MJCF)
    with pytest.raises(RuntimeError, match="link2"):
        common.Pin(str(path), "tip", False, base_frame="link2")


def test_kinematics_joint_limits_override_model_limits():
    """Given joint limits replace the model limits, so solutions stay inside a range narrower than the MJCF."""
    robot = rcs.ROBOTS[common.RobotType.FR3]
    narrow_limits = np.array(
        [
            [-2.3476, -1.5454, -2.4937, -2.7714, -2.5100, 0.7773, -2.7045],
            [2.3476, 1.5454, 2.4937, -0.4226, 2.5100, 4.2841, 2.7045],
        ]
    )
    pin_model_limits = common.Pin(robot.mjcf_model_path, robot.attachment_site, False)
    pin_narrow_limits = common.Pin(robot.mjcf_model_path, robot.attachment_site, False, joint_limits=narrow_limits)
    lower, upper = narrow_limits
    q_target = robot.q_home.copy()
    q_target[0] = upper[0] + 0.3
    pose = pin_model_limits.forward(q_target, NO_TCP_OFFSET)
    q_seed = robot.q_home.copy()
    q_seed[0] = upper[0]

    q_model: np.ndarray | None = pin_model_limits.inverse(pose, q_seed, NO_TCP_OFFSET)
    q_narrow: np.ndarray | None = pin_narrow_limits.inverse(pose, q_seed, NO_TCP_OFFSET)

    assert q_model is not None and q_model[0] > upper[0]
    assert q_narrow is not None
    assert np.all((q_narrow >= lower) & (q_narrow <= upper))
    assert pin_narrow_limits.forward(q_narrow, NO_TCP_OFFSET).is_close(pose, eps_r=1e-3, eps_t=1e-3)


def test_kinematics_joint_limits_more_joints_than_model_raises():
    robot = rcs.ROBOTS[common.RobotType("UR5e")]
    too_many = np.vstack([-np.ones(robot.dof + 1), np.ones(robot.dof + 1)])
    with pytest.raises(ValueError, match="joint_limits"):
        common.Pin(robot.mjcf_model_path, robot.attachment_site, False, joint_limits=too_many)
