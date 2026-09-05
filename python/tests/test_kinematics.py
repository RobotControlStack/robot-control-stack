import numpy as np
import pytest

import rcs
from rcs import common

# Restrict this test to robot models that are expected to work through the generic Pin binding.
# Panda currently segfaults in the native binding here, and SO101 uses a dedicated IK implementation.
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
