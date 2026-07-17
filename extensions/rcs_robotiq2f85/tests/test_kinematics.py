from pathlib import Path

import mujoco
import numpy as np
import pytest

import rcs
from rcs import common
from rcs_robotiq2f85.kinematics import robotiq_2f85_finger_pose_offsets_from_sites, robotiq_2f85_finger_poses

ROBOTIQ_2F85_MODEL_PATH = (
    Path(__file__).resolve().parents[3]
    / "assets"
    / "grippers"
    / "robotiq_2f85"
    / "robotiq_2f85.xml"
)

@pytest.mark.parametrize(
    ("normalized_command", "left_position", "right_position"),
    [
        (0.0, [-0.007465, -0.00905, 0.129130], [0.007466, 0.00905, 0.129130]),
        (0.5, [-0.030551, -0.00905, 0.126510], [0.030551, 0.00905, 0.126510]),
        (1.0, [-0.050800, -0.00905, 0.114817], [0.050800, 0.00905, 0.114817]),
    ],
)
def test_robotiq_2f85_fingertip_fk(normalized_command, left_position, right_position):
    """Look up follower FK from a normalized command: 0 closed, 1 open."""
    poses = robotiq_2f85_finger_poses(normalized_command, model_path=ROBOTIQ_2F85_MODEL_PATH)

    expected_left_pose = common.Pose(
        rotation=np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]),
        translation=np.array(left_position),
    )
    expected_right_pose = common.Pose(
        rotation=np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]),
        translation=np.array(right_position),
    )

    assert poses.left.is_close(expected_left_pose, eps_r=1e-3, eps_t=1e-4)
    assert poses.right.is_close(expected_right_pose, eps_r=1e-3, eps_t=1e-4)


def test_robotiq_2f85_fingertip_fk_with_custom_mount_offsets():
    # The fixed body/site transforms match robotiq_2f85_digit_new.xml.
    custom_mount_model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="left_follower">
              <body name="left_digit" euler="-1.571 0 0" pos="-0.0112 0.032 -0.00905">
                <site name="left_digit_pad" pos="0.02 0 0.016" euler="3.142 1.571 0"/>
              </body>
            </body>
            <body name="right_follower">
              <body name="right_digit" euler="-1.571 0 0" pos="-0.0112 0.032 -0.00905">
                <site name="right_digit_pad" pos="0.02 0 0.016" euler="3.142 1.571 0"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    custom_mount_data = mujoco.MjData(custom_mount_model)
    offsets = robotiq_2f85_finger_pose_offsets_from_sites(
        custom_mount_model,
        custom_mount_data,
        left_site="left_digit_pad",
        right_site="right_digit_pad",
    )

    poses = robotiq_2f85_finger_poses(0.5, offsets=offsets, model_path=ROBOTIQ_2F85_MODEL_PATH)

    expected_left_pose = common.Pose(
        rotation=np.array(
            [
                [-0.0001713, 0.0, 1.0],
                [-0.0002037, -1.0, 0.0],
                [1.0, -0.0002037, 0.0001713],
            ]
        ),
        translation=np.array([-0.021750, 0.0000033, 0.174510]),
    )
    expected_right_pose = common.Pose(
        rotation=np.array(
            [
                [0.0002066, 0.0, -1.0],
                [0.0002037, 1.0, 0.0],
                [1.0, -0.0002037, 0.0002066],
            ]
        ),
        translation=np.array([0.021752, -0.0000033, 0.174510]),
    )

    assert poses.left.is_close(expected_left_pose, eps_r=1e-3, eps_t=1e-4)
    assert poses.right.is_close(expected_right_pose, eps_r=1e-3, eps_t=1e-4)
