import math

import pytest
from rcsss import common
import numpy as np


class TestPose:
    """
    This class tests the methods of the Pose class and its multiple constructors.
    """

    @pytest.fixture
    def identity_pose(self):
        """This fixture can be reused wherever if no transformation pose is needed"""
        return common.Pose()

    def test_rotation_q(self, identity_pose: common.Pose):
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
    def test_interpolate_r_t(self, initial_pose_rotation_m: np.ndarray, initial_pose_translation: np.ndarray,
                             destination_pose_rotation_m: np.ndarray, destination_pose_translation: np.ndarray,
                             progress: float, expected_pose_rotation_m: np.ndarray, expected_pose_translation: np.ndarray):
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
    def test_is_close(self, pose1_array: np.ndarray, pose2_array: np.ndarray, eps: float, expected_bool: bool):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: there is a term with (1.3 -1) = 0.2 diff and hence with an absolute tolerance of 0.1, the two poses
                   are considered to be 'not' close to each other.
        """
        pose1 = common.Pose(pose_matrix=pose1_array)
        pose2 = common.Pose(pose_matrix=pose2_array)
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
    def test_multiply(self, pose1_array: np.ndarray, pose2_array: np.ndarray, expected_pose_array: np.ndarray):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: pose1_array and pose2_array are pure translations, when multiplied they should lead to the pose array
                   containing the combined translations.
        """
        pose1 = common.Pose(pose_matrix=pose1_array)
        pose2 = common.Pose(pose_matrix=pose2_array)

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
    def test_inverse(self, pose_array: np.ndarray, expected_pose_array: np.ndarray):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: We have a pure translation and the expected inverse should negate the translations in the matrix.
        """
        pose1 = common.Pose(pose_matrix=pose_array)
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
    def test_pose_matrix(self, quaternion: np.ndarray, translation_vector: np.ndarray, expected_pose_matrix: np.ndarray):
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
    def test_rotation_rpy(self, pose_m: np.ndarray, expected_rpy: common.RPY):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: The 'no rotation pose_m' should lead to all roll, pitch, yaw being zero.
        """
        pose = common.Pose(pose_matrix=pose_m)
        rpy = pose.rotation_rpy()
        for ang in ['roll', 'pitch', 'yaw']:
            out_angle = getattr(rpy, ang)
            exp_angle = getattr(expected_rpy, ang)
            assert math.isclose(out_angle, exp_angle, abs_tol=1e-8)

    def test_rpy_conversion(self):
        # equal to identity
        assert common.Pose(translation=np.array([0, 0, 0]), rpy_vector=np.array([0, 0, 0])).is_close(common.Pose())

        home_m = np.array(
            [
                [9.99999352e-01, 2.51302265e-05, -1.13823380e-03, 3.06764031e-01],
                [2.65946429e-05, -9.99999172e-01, 1.28657214e-03, 1.39119827e-04],
                [-1.13820053e-03, -1.28660158e-03, -9.99998525e-01, 4.86190811e-01],
                [0.00000000e00, 0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )

        home_pose = common.Pose(pose_matrix=home_m)

        assert np.allclose(home_pose.pose_matrix(), home_m)

        trpy = home_pose.xyzrpy()
        assert np.allclose(trpy[:3], home_pose.translation())

        home_pose2 = common.Pose(translation=trpy[:3], rpy_vector=trpy[3:])
        home_m2 = home_pose2.pose_matrix()

        assert home_pose.is_close(home_pose2)
        assert np.allclose(home_m, home_m2)

    @pytest.mark.parametrize(
        ("pose_m", "expected_translation_m"),
        [
            (
                np.array([[1.0, 0, 0, 1.0], [0, 1.0, 0, 2.0], [0, 0, 1.0, 3.0], [0, 0, 0, 1.0]]),
                np.array([1.0, 2.0, 3.0])
            )
        ])
    def test_translation(self, pose_m: np.ndarray, expected_translation_m: np.ndarray):
        """
        Here the Pose class is initiated using a single 4x4 matrix where the rotation and translation are combined.
        TestCase1: The simple pose_m should yield the expected translation vector
        """
        pose = common.Pose(pose_matrix=pose_m)
        out_translation_v = pose.translation()
        assert np.array_equal(expected_translation_m, out_translation_v)
