from typing import cast

import numpy as np
import pytest
import rcsss
from rcsss.envs.base import (
    ControlMode,
    GripperDictType,
    JointsDictType,
    RobotEnv,
    TQuatDictType,
    TRPYDictType,
)
from rcsss.envs.creators import RCSSimEnvCreator
from rcsss.envs.utils import (
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
)


@pytest.fixture()
def cfg():
    return default_fr3_sim_robot_cfg()


@pytest.fixture()
def gripper_cfg():
    return default_fr3_sim_gripper_cfg()


@pytest.fixture()
def cam_cfg():
    return default_mujoco_cameraset_cfg()


class TestSimEnvs:
    """This class is for testing common sim env functionalities"""

    def assert_no_pose_change(self, info: dict, initial_obs: dict, final_obs: dict):
        assert info["ik_success"]
        assert info["is_sim_converged"]
        out_pose = rcsss.common.Pose(
            translation=np.array(final_obs["tquat"][:3]), quaternion=np.array(final_obs["tquat"][3:])
        )
        expected_pose = rcsss.common.Pose(
            translation=np.array(initial_obs["tquat"][:3]), quaternion=np.array(initial_obs["tquat"][3:])
        )
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)

    def assert_collision(self, info: dict):
        assert info["ik_success"]
        assert info["is_sim_converged"]
        assert info["collision"]


class TestSimEnvsTRPY(TestSimEnvs):
    """This class is for testing TRPY sim env functionalities"""

    def test_reset(self, cfg, gripper_cfg, cam_cfg):
        """
        Test reset functionality.
        """
        # TODO:
        # - test initial pose after reset.
        # - test initial gripper config.
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TRPY, cfg, gripper_cfg=gripper_cfg, camera_set_cfg=cam_cfg, max_relative_movement=None
        )
        # Test double reset. Regression test. A lot can go wrong when resetting.
        env.reset()
        env.reset()

    def test_zero_action_trpy(self, cfg):
        """
        Test that a zero action does not change the state significantly
        """
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TRPY, cfg, gripper_cfg=None, camera_set_cfg=None, max_relative_movement=None
        )
        obs_initial, _ = env.reset()
        zero_action = TRPYDictType(xyzrpy=obs_initial["xyzrpy"])
        obs, _, _, _, info = env.step(zero_action)
        self.assert_no_pose_change(info, obs_initial, obs)

    def test_non_zero_action_trpy(self, cfg):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TRPY, cfg, gripper_cfg=None, camera_set_cfg=None, max_relative_movement=None
        )
        obs_initial, _ = env.reset()
        # action to be performed
        x_pos_change = 0.2
        initial_tquat = obs_initial["tquat"].copy()
        t = initial_tquat[:3]
        # shift the translation in x
        t[0] += x_pos_change
        q = initial_tquat[3:]
        initial_pose = rcsss.common.Pose(translation=np.array(t), quaternion=np.array(q))
        xyzrpy = np.concatenate([t, initial_pose.rotation_rpy().as_vector()], axis=0)
        non_zero_action = TRPYDictType(xyzrpy=np.array(xyzrpy))
        non_zero_action.update(GripperDictType(gripper=0))
        expected_obs = obs_initial.copy()
        expected_obs["tquat"][0] += x_pos_change
        obs, _, _, _, info = env.step(non_zero_action)
        self.assert_no_pose_change(info, expected_obs, obs)

    def test_relative_zero_action_trpy(self, cfg, gripper_cfg):
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TRPY, cfg, gripper_cfg=gripper_cfg, camera_set_cfg=None, max_relative_movement=0.5
        )
        obs_initial, _ = env.reset()
        # action to be performed
        zero_action = TRPYDictType(xyzrpy=np.array([0, 0, 0, 0, 0, 0], dtype=np.float32))
        zero_action.update(GripperDictType(gripper=0))
        obs, _, _, _, info = env.step(zero_action)
        self.assert_no_pose_change(info, obs_initial, obs)

    def test_relative_non_zero_action(self, cfg, gripper_cfg):
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TRPY, cfg, gripper_cfg=gripper_cfg, camera_set_cfg=None, max_relative_movement=0.5
        )
        obs_initial, _ = env.reset()
        # action to be performed
        x_pos_change = 0.2
        non_zero_action = TRPYDictType(xyzrpy=np.array([x_pos_change, 0, 0, 0, 0, 0]))
        non_zero_action.update(GripperDictType(gripper=0))
        expected_obs = obs_initial.copy()
        expected_obs["tquat"][0] += x_pos_change
        obs, _, _, _, info = env.step(non_zero_action)
        self.assert_no_pose_change(info, obs_initial, expected_obs)

    def test_collision_trpy(self, cfg, gripper_cfg):
        """
        Check that an obvious collision is detected by sim
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TRPY, cfg, gripper_cfg=gripper_cfg, camera_set_cfg=None, max_relative_movement=None
        )
        obs, _ = env.reset()
        # an obvious below ground collision action
        obs["xyzrpy"][0] = 0.4
        obs["xyzrpy"][2] = -0.05
        collision_action = TRPYDictType(xyzrpy=obs["xyzrpy"])
        collision_action.update(GripperDictType(gripper=0))
        obs, _, _, _, info = env.step(collision_action)
        self.assert_collision(info)

    def test_collision_guard_trpy(self, cfg, gripper_cfg):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TRPY,
            cfg,
            gripper_cfg=gripper_cfg,
            collision_guard=True,
            camera_set_cfg=None,
            max_relative_movement=None,
        )
        obs, _ = env.reset()
        unwrapped = cast(RobotEnv, env.unwrapped)
        p1 = unwrapped.robot.get_joint_position()
        # an obvious below ground collision action
        obs["xyzrpy"][0] = 0.4
        obs["xyzrpy"][2] = -0.05
        collision_action = TRPYDictType(xyzrpy=obs["xyzrpy"])
        collision_action.update(GripperDictType(gripper=0))
        _, _, _, _, info = env.step(collision_action)
        p2 = unwrapped.robot.get_joint_position()
        self.assert_collision(info)
        # assure that the robot did not move
        assert np.allclose(p1, p2)


class TestSimEnvsTquat(TestSimEnvs):
    """This class is for testing Tquat sim env functionalities"""

    def test_reset(self, cfg, gripper_cfg, cam_cfg):
        """
        Test reset functionality.
        """
        # TODO:
        # - test initial pose after reset.
        # - test initial gripper config.
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TQuat,
            cfg,
            gripper_cfg=gripper_cfg,
            camera_set_cfg=cam_cfg,
            max_relative_movement=None,
        )
        # Test double reset. Regression test. A lot can go wrong when resetting.
        env.reset()
        env.reset()

    def test_non_zero_action_tquat(self, cfg):
        """
        Test that a zero action does not change the state significantly in the tquat configuration
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TQuat, cfg, gripper_cfg=None, camera_set_cfg=None, max_relative_movement=None
        )
        obs_initial, _ = env.reset()
        # action to be performed
        t = obs_initial["tquat"][:3]
        q = obs_initial["tquat"][3:]
        x_pos_change = 0.3
        # updating the x action by 30cm
        t[0] += x_pos_change
        non_zero_action = TQuatDictType(tquat=np.concatenate([t, q], axis=0))
        expected_obs = obs_initial.copy()
        expected_obs["tquat"][0] += x_pos_change
        _, _, _, _, info = env.step(non_zero_action)
        self.assert_no_pose_change(info, obs_initial, expected_obs)

    def test_zero_action_tquat(self, cfg):
        """
        Test that a zero action does not change the state significantly in the tquat configuration
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TQuat, cfg, gripper_cfg=None, camera_set_cfg=None, max_relative_movement=None
        )
        obs_initial, info_ = env.reset()
        home_action_vec = obs_initial["tquat"]
        zero_action = TQuatDictType(tquat=home_action_vec)
        obs, _, _, _, info = env.step(zero_action)
        self.assert_no_pose_change(info, obs_initial, obs)

    def test_relative_zero_action_tquat(self, cfg, gripper_cfg):
        # env creation
        env_rel = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TQuat,
            cfg,
            gripper_cfg=gripper_cfg,
            camera_set_cfg=None,
            max_relative_movement=0.5,
        )
        obs_initial, _ = env_rel.reset()
        zero_rel_action = TQuatDictType(tquat=np.array([0, 0, 0, 0, 0, 0, 1.0], dtype=np.float32))
        zero_rel_action.update(GripperDictType(gripper=0))
        obs, _, _, _, info = env_rel.step(zero_rel_action)
        self.assert_no_pose_change(info, obs_initial, obs)

    def test_collision_tquat(self, cfg, gripper_cfg):
        """
        Check that an obvious collision is detected by sim
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TQuat,
            cfg,
            gripper_cfg=gripper_cfg,
            camera_set_cfg=None,
            max_relative_movement=None,
        )
        obs, _ = env.reset()
        # an obvious below ground collision action
        obs["tquat"][0] = 0.4
        obs["tquat"][2] = -0.05
        collision_action = TQuatDictType(tquat=obs["tquat"])
        collision_action.update(GripperDictType(gripper=0))
        _, _, _, _, info = env.step(collision_action)
        self.assert_collision(info)

    def test_collision_guard_tquat(self, cfg, gripper_cfg):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.CARTESIAN_TQuat,
            cfg,
            gripper_cfg=gripper_cfg,
            collision_guard=True,
            camera_set_cfg=None,
            max_relative_movement=None,
        )
        obs, _ = env.reset()
        unwrapped = cast(RobotEnv, env.unwrapped)
        p1 = unwrapped.robot.get_joint_position()
        # an obvious below ground collision action
        obs["tquat"][0] = 0.4
        obs["tquat"][2] = -0.05
        collision_action = TQuatDictType(tquat=obs["tquat"])
        collision_action.update(GripperDictType(gripper=0))
        _, _, _, _, info = env.step(collision_action)
        p2 = unwrapped.robot.get_joint_position()
        self.assert_collision(info)
        # assure that the robot did not move
        assert np.allclose(p1, p2)


class TestSimEnvsJoints(TestSimEnvs):
    """This class is for testing Joints sim env functionalities"""

    def test_reset(self, cfg, gripper_cfg, cam_cfg):
        """
        Test reset functionality.
        """
        # TODO:
        # - test initial pose after reset.
        # - test initial gripper config.
        env = RCSSimEnvCreator()(
            ControlMode.JOINTS, cfg, gripper_cfg=gripper_cfg, camera_set_cfg=cam_cfg, max_relative_movement=None
        )
        # Test double reset. Regression test. A lot can go wrong when resetting.
        env.reset()
        env.reset()

    def test_zero_action_joints(self, cfg):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.JOINTS, cfg, gripper_cfg=None, camera_set_cfg=None, max_relative_movement=None
        )
        obs_initial, _ = env.reset()
        # action to be performed
        zero_action = JointsDictType(joints=np.array(obs_initial["joints"]))
        obs, _, _, _, info = env.step(zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]
        assert np.allclose(obs["joints"], obs_initial["joints"], atol=0.01, rtol=0)

    def test_non_zero_action_joints(self, cfg):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.JOINTS, cfg, gripper_cfg=None, camera_set_cfg=None, max_relative_movement=None
        )
        obs_initial, _ = env.reset()
        new_joint_vals = obs_initial["joints"] + np.array([0.1, 0.1, 0.1, 0.1, -0.1, -0.1, 0.1], dtype=np.float32)
        # action to be performed
        non_zero_action = JointsDictType(joints=new_joint_vals)
        obs, _, _, _, info = env.step(non_zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]
        assert np.allclose(obs["joints"], non_zero_action["joints"], atol=0.01, rtol=0)

    def test_collision_joints(self, cfg, gripper_cfg):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.JOINTS, cfg, gripper_cfg=gripper_cfg, camera_set_cfg=None, max_relative_movement=None
        )
        env.reset()
        # the below action is a test_case where there is an obvious collision regardless of the gripper action
        collision_act = JointsDictType(joints=np.array([0, 1.78, 0, -1.45, 0, 0, 0], dtype=np.float32))
        collision_act.update(GripperDictType(gripper=1))
        _, _, _, _, info = env.step(collision_act)
        self.assert_collision(info)

    def test_collision_guard_joints(self, cfg, gripper_cfg):
        """
        Check that an obvious collision is detected by sim
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.JOINTS,
            cfg,
            gripper_cfg=gripper_cfg,
            collision_guard=True,
            camera_set_cfg=None,
            max_relative_movement=None,
        )
        env.reset()
        unwrapped = cast(RobotEnv, env.unwrapped)
        p1 = unwrapped.robot.get_joint_position()
        # the below action is a test_case where there is an obvious collision regardless of the gripper action
        collision_act = JointsDictType(joints=np.array([0, 1.78, 0, -1.45, 0, 0, 0], dtype=np.float32))
        collision_act.update(GripperDictType(gripper=1))
        _, _, _, _, info = env.step(collision_act)
        p2 = unwrapped.robot.get_joint_position()

        self.assert_collision(info)
        # assure that the robot did not move
        assert np.allclose(p1, p2)

    def test_relative_zero_action_joints(self, cfg, gripper_cfg):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """
        # env creation
        env = RCSSimEnvCreator()(
            ControlMode.JOINTS, cfg, gripper_cfg=gripper_cfg, camera_set_cfg=None, max_relative_movement=0.5
        )
        obs_initial, _ = env.reset()
        act = JointsDictType(joints=np.array([0, 0, 0, 0, 0, 0, 0], dtype=np.float32))
        act.update(GripperDictType(gripper=1))
        obs, _, _, _, info = env.step(act)
        self.assert_no_pose_change(info, obs_initial, obs)
