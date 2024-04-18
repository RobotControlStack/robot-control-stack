import pytest
from rcsss import common
import numpy as np


@pytest.mark.parametrize(
    "initial_pose, destination_pose, progress, expected_pose_matrix",
    [
        [
            np.array([[1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]),
            np.array([[1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0, 1.0]]),
            1.0,
            np.array(
                [
                    [-0.33333333, 0.66666667, 0.66666667, 1],
                    [0.66666667, -0.33333333, 0.66666667, 1.0],
                    [0.66666667, 0.66666667, -0.33333333, 1.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        ]
    ],
)
def test_interpolate(initial_pose, destination_pose, progress, expected_pose_matrix):
    start_pose = common.Pose(initial_pose)
    end_pose = common.Pose(destination_pose)
    result = start_pose.interpolate(end_pose, progress=progress)
    assert np.allclose(result.pose_matrix(), expected_pose_matrix)
