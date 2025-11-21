import numpy as np
import pytest

import rcs
from rcs import common

# Map robot types to their end-effector frame names
ROBOT_FRAME_IDS = {
    common.RobotType.FR3: "attachment_site_0",
    common.RobotType.Panda: "attachment_site_0",
    common.RobotType.XArm7: "attachment_site",
    common.RobotType.SO101: "gripper",
    common.RobotType.UR5e: "attachment_site",
}


# only for scene that have empty_world in key
@pytest.mark.parametrize("scene_name", [k for k in rcs.scenes if "empty_world" in k])
def test_kinematics_identity(scene_name):
    scene = rcs.scenes[scene_name]

    # Determine model path and type
    model_path = scene.mjcf_robot

    # Get frame ID
    if scene.robot_type not in ROBOT_FRAME_IDS:
        pytest.skip(f"Frame ID not defined for robot type {scene.robot_type}")

    frame_id = ROBOT_FRAME_IDS[scene.robot_type]

    # Initialize Pinocchio interface
    try:
        pin = common.Pin(model_path, frame_id, False)
    except Exception as e:
        pytest.fail(f"Failed to initialize Pin for {scene_name}: {e}")

    # Get home configuration
    try:
        meta_config = common.robots_meta_config(scene.robot_type)
        q_home = meta_config.q_home
    except Exception as e:
        pytest.fail(f"Failed to get meta config for {scene_name}: {e}")

    # Test 1: FK at home
    # Identity pose (no TCP offset)
    tcp_offset = common.Pose()

    pose_home = pin.forward(q_home, tcp_offset)
    assert isinstance(pose_home, common.Pose)

    # Test 2: IK at home pose should return a solution (ideally close to q_home, but IK is redundant)
    # We use q_home as initial guess
    q_sol = pin.inverse(pose_home, q_home, tcp_offset)

    assert q_sol is not None, "IK failed for home pose"

    # Verify the solution with FK
    pose_sol = pin.forward(q_sol, tcp_offset)

    # Check if pose_sol is close to pose_home
    assert pose_sol.is_close(
        pose_home, eps_r=1e-4, eps_t=1e-4
    ), f"FK(IK(pose)) does not match pose.\nOriginal: {pose_home}\nResult: {pose_sol}"

    # Test 3: Perturbed configuration
    # Add small noise to q_home to test non-trivial pose
    # Ensure we stay within limits if possible, but for small noise it should be fine
    np.random.seed(42)
    q_perturbed = q_home + np.random.uniform(-0.1, 0.1, size=q_home.shape)

    pose_perturbed = pin.forward(q_perturbed, tcp_offset)  # type: ignore
    q_sol_perturbed = pin.inverse(pose_perturbed, q_home, tcp_offset)  # Use q_home as seed

    assert q_sol_perturbed is not None, "IK failed for perturbed pose"

    pose_sol_perturbed = pin.forward(q_sol_perturbed, tcp_offset)
    assert pose_sol_perturbed.is_close(
        pose_perturbed, eps_r=1e-3, eps_t=1e-3
    ), f"FK(IK(perturbed_pose)) does not match.\nOriginal: {pose_perturbed}\nResult: {pose_sol_perturbed}"
