import math

import pytest
from rcsss import common
import numpy as np


class TestPose:
    """
    This script tests the methods of the Pose class and its multiple constructors.
    """

    @pytest.fixture
    def identity_pose(self):
        """This fixture can be reused wherever if no transformation pose is needed"""
        return common.Pose()

    def test_rotation_q(self, identity_pose):
        """
        Here the Pose class initiated using the rpy object and the translation vector.
        TestCase1: The quaternion corresponding to the identity pose should be expected_quaternion1
        """
        out_q = identity_pose.rotation_q()
        expected_quaternion1 = np.array([0, 0, 0, 1])
        assert np.array_equal(out_q, expected_quaternion1)

    @pytest.mark.parametrize(
        ("initial_pose_rotation_m", "initial_pose_translation",
         "destination_pose_rotation_m", "destination_pose_translation",
         "progress", "expected_pose_rotation_m", "expected_pose_translation"),
        [
            (
                np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]),
                np.array([[0], [0], [0]]),
                np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]),
                np.array([[1.0], [1.0], [1.0]]),
                1.0,
                np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]),
                np.array([1.0, 1.0, 1.0]),
            )
        ],
    )
    def test_interpolate_r_t(self, initial_pose_rotation_m: np.array, initial_pose_translation: np.array,
                             destination_pose_rotation_m: np.array, destination_pose_translation: np.array,
                             progress: float, expected_pose_rotation_m: np.array, expected_pose_translation: np.array):
        """
        Here the Pose class initiated using the rotation and the translation matrices is used.
        TestCase1: with progress=1, the interpolated pose should be equal to the start pose
        """
        start_pose = common.Pose(rotation=initial_pose_rotation_m, translation=initial_pose_translation)
        end_pose = common.Pose(rotation=destination_pose_rotation_m, translation=destination_pose_translation)
        result = start_pose.interpolate(end_pose, progress=progress)
        assert np.array_equal(result.rotation_m(), expected_pose_rotation_m)
        assert np.array_equal(result.translation(), expected_pose_translation)

    @pytest.mark.parametrize(
        ("pose1_array", "pose2_array", "eps", "expected_bool"),
        [
            (
                np.array([[1.0, 0, 0, 1.0], [0, 1.0, 0, 2.0], [0, 0, 1.0, 3.0], [0, 0, 0, 1.0]]),
                np.array([[1.0, 0, 0, 1.1], [0, 1.0, 0, 2.0], [0, 0, 1.0, 3.0], [0, 0, 0, 1.0]]),
                0.1,
                False
            ),
            (
                np.array([[1.0, 0, 0, 1.0], [0, 1.0, 0, 2.0], [0, 0, 1.0, 3.0], [0, 0, 0, 1.0]]),
                np.array([[1.0, 0, 0, 1.09], [0, 1.0, 0, 2.0], [0, 0, 1.0, 3.0], [0, 0, 0, 1.0]]),
                0.1,
                True
            ),
            (
                np.array([[1.0, 1e-8, 0, 0.0], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0, 0, 0, 1.0]]),
                np.array([[1.0, 0, 0, 0.0], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0, 0, 0, 1.0]]),
                0.1,
                True
            ),
            (
                np.array([[1.0, 1, 0, 0.0], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0, 0, 0, 1.0]]),
                np.array([[1.0, 0, 0, 0.0], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0, 0, 0, 1.0]]),
                0.1,
                False
            ),
        ],
    )
    def test_is_close(self, pose1_array: np.array, pose2_array: np.array, eps: float, expected_bool: bool):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: there is a term with (1.3 -1) = 0.2 diff and hence with an absolute tolerance of 0.1, the two poses
                   are considered to be 'not' close to each other.
        """
        pose1 = common.Pose(pose1_array)
        pose2 = common.Pose(pose2_array)
        out_bool = pose1.is_close(pose2, eps_t=eps)
        print(f"{out_bool = }")
        assert out_bool == expected_bool

    @pytest.mark.parametrize(
        ("pose1_array", "pose2_array", "expected_pose_array"),
        [
            (
                np.array([[1.0, 0, 0, 1], [0, 1.0, 0, 2], [0, 0, 1.0, 3], [0, 0, 0, 1]]),
                np.array([[1.0, 0, 0, 4], [0, 1.0, 0, 5], [0, 0, 1.0, 6], [0, 0, 0, 1]]),
                np.array([[1.0, 0, 0, 5], [0, 1.0, 0, 7], [0, 0, 1.0, 9], [0, 0, 0, 1]])
            ),
        ],
    )
    def test_multiply(self, pose1_array: np.array, pose2_array: np.array, expected_pose_array: np.array):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: pose1_array and pose2_array are pure translations, when multiplied they should lead to the pose array
                   containing the combined translations.
        """
        pose1 = common.Pose(pose1_array)
        pose2 = common.Pose(pose2_array)

        out_pose = pose1 * pose2
        assert np.array_equal(out_pose.pose_matrix(), expected_pose_array)

    @pytest.mark.parametrize(
        ("pose_array", "expected_pose_array"),
        [
            (
                np.array([[1.0, 0, 0, 1], [0, 1.0, 0, 2], [0, 0, 1.0, 3], [0, 0, 0, 1]]),
                np.array([[1.0, 0, 0, -1], [0, 1.0, 0, -2], [0, 0, 1.0, -3], [0, 0, 0, 1]])
            ),
        ],
    )
    def test_inverse(self, pose_array: np.array, expected_pose_array: np.array):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: We have a pure translation and the expected inverse should negate the translations in the matrix.
        """
        pose1 = common.Pose(pose_array)
        out_pose = pose1.inverse()
        assert np.array_equal(out_pose.pose_matrix(), expected_pose_array)

    @pytest.mark.parametrize(
        ("quaternion", "translation_vector", "expected_pose_matrix"),
        [
            (
                np.array([0, 0, 0, 1.0]),
                np.array([1.0, 1.0, 1.0]),
                np.array([[1.0, 0, 0, 1.0], [0, 1.0, 0, 1.0], [0, 0, 1.0, 1.0], [0, 0, 0, 1.0]])
            ),
        ],
    )
    def test_pose_matrix(self, quaternion: np.array, translation_vector: np.array, expected_pose_matrix: np.array):
        """
        Here the Pose class is initiated using a quaternion and a translation vector.
        TestCase1: The 'no rotation quaternion' and a unit translation in all three directions is used here.
        """
        pose = common.Pose(quaternion=quaternion, translation=translation_vector)
        out_pose_matrix = pose.pose_matrix()
        print(f"{out_pose_matrix = }")
        assert np.array_equal(out_pose_matrix, expected_pose_matrix)

    @pytest.mark.parametrize(
        ("pose_m", "expected_rpy"),
        [
            (
                np.array([[1.0, 0, 0, 1.0], [0, 1.0, 0, 1.0], [0, 0, 1.0, 1.0], [0, 0, 0, 1.0]]),
                common.RPY(roll=0.0, pitch=0.0, yaw=0.0)
            )
        ])
    def test_rotation_rpy(self, pose_m, expected_rpy):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: The 'no rotation pose_m' should lead to all roll, pitch, yaw being zero.
        """
        pose = common.Pose(pose=pose_m)
        rpy = pose.rotation_rpy()
        for ang in ['roll', 'pitch', 'yaw']:
            out_angle = getattr(rpy, ang)
            exp_angle = getattr(expected_rpy, ang)
            assert math.isclose(out_angle, exp_angle, abs_tol=1e-8)

    @pytest.mark.parametrize(
        ("pose_m", "expected_translation_m"),
        [
            (
                np.array([[1.0, 0, 0, 1.0], [0, 1.0, 0, 2.0], [0, 0, 1.0, 3.0], [0, 0, 0, 1.0]]),
                np.array([1.0, 2.0, 3.0])
            )
        ])
    def test_translation(self, pose_m, expected_translation_m):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: The simple pose_m should yield the expected translation vector
        """
        pose = common.Pose(pose=pose_m)
        out_translation_v = pose.translation()
        assert np.array_equal(expected_translation_m, out_translation_v)
