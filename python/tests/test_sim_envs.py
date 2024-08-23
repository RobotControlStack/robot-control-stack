from collections import OrderedDict

import numpy as np
import pytest
import rcsss
from rcsss import sim
from rcsss._core import common
from rcsss.camera.sim import SimCameraSetConfig
from rcsss.envs import typed_factory as tf


@pytest.fixture
def cfg():
    fr3_config = sim.FR3Config()
    fr3_config.ik_duration_in_milliseconds = 300
    fr3_config.realtime = False
    return fr3_config


@pytest.fixture
def gripper_cfg():
    gripper_cfg = sim.FHConfig()
    return gripper_cfg


@pytest.fixture
def cam_cfg():
    # no camera obs needed for this test
    cameras = {}
    cam_cfg_ = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=50)
    return cam_cfg_


class TestSimEnvs:
    """This class is for testing common sim env functionalities"""

    mjcf_path = str(rcsss.scenes["fr3_empty_world"])
    urdf_path = str(rcsss.scenes["lab"].parent / "fr3.urdf")

    def test_model_availability(self):
        assert (
            "lab" in rcsss.scenes
        ), "This pip package was not built with the UTN lab models which is needed for these tests, aborting."


class TestSimEnvsTRPY:
    """This class is for testing TRPY sim env functionalities"""

    def test_zero_action_trpy(self, cfg):
        """
        Test that a zero action does not change the state significantly
        """
        env, _ = tf.produce_env_sim_trpy(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, robot_id="0")
        obs_initial, _ = env.reset()

        t = obs_initial["tquart"][:3]
        q = obs_initial["tquart"][3:]
        initial_pose = common.Pose(translation=np.array(t), quaternion=np.array(q))
        xyzrpy = t.tolist() + initial_pose.rotation_rpy().as_vector().tolist()
        zero_action = OrderedDict([("xyzrpy", xyzrpy)])
        obs, _, _, _, info = env.step(zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]

        out_pose = common.Pose(translation=np.array(obs["tquart"][:3]), quaternion=np.array(obs["tquart"][3:]))
        expected_pose = initial_pose
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)

    def test_non_zero_action_trpy(self, cfg):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        # env creation
        env, _ = tf.produce_env_sim_trpy(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, robot_id="0")
        obs_initial, _ = env.reset()
        # action to be performed`
        x_pos_change = 0.2
        initial_tquart = obs_initial["tquart"].copy()
        t = initial_tquart[:3]
        # shift the translation in x
        t[0] += x_pos_change
        q = initial_tquart[3:]
        initial_pose = common.Pose(translation=np.array(t), quaternion=np.array(q))
        xyzrpy = t.tolist() + initial_pose.rotation_rpy().as_vector().tolist()
        non_zero_action = OrderedDict([("xyzrpy", np.array(xyzrpy)), ("gripper", 0)])
        expected_obs = obs_initial["tquart"].copy()
        expected_obs[0] += x_pos_change
        obs, _, _, _, info = env.step(non_zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]

        out_pose = common.Pose(translation=np.array(obs["tquart"][:3]), quaternion=np.array(obs["tquart"][3:]))
        expected_pose = common.Pose(translation=np.array(expected_obs[:3]), quaternion=np.array(expected_obs[3:]))
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)

    def test_relative_zero_action_trpy(self, cfg, gripper_cfg, cam_cfg):
        # env creation
        env = tf.produce_env_sim_trpy_gripper_camera_rel(
            TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, gripper_cfg, cam_cfg, robot_id="0"
        )
        obs_initial, _ = env.reset()

        # action to be performed
        zero_action = OrderedDict([("xyzrpy", np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)), ("gripper", 0)])
        obs, _, _, _, info = env.step(zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]

        out_pose = common.Pose(translation=np.array(obs["tquart"][:3]), quaternion=np.array(obs["tquart"][3:]))
        expected_pose = common.Pose(
            translation=np.array(obs_initial["tquart"][:3]), quaternion=np.array(obs_initial["tquart"][3:])
        )
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)

    def test_relative_non_zero_action(self, cfg, gripper_cfg, cam_cfg):
        # env creation
        env = tf.produce_env_sim_trpy_gripper_camera_cg_rel(
            TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, gripper_cfg, cam_cfg, robot_id="0"
        )
        obs_initial, _ = env.reset()

        # action to be performed
        x_pos_change = 0.2
        non_zero_action = OrderedDict(
            [
                ("xyzrpy", np.array([x_pos_change, 0, 0, 0, 0, 0], dtype=np.float32)),
                ("gripper", 0),
            ]
        )
        expected_obs = obs_initial["tquart"].copy()

        expected_obs[0] += x_pos_change
        obs, _, _, _, info = env.step(non_zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]

        out_pose = common.Pose(translation=np.array(obs["tquart"][:3]), quaternion=np.array(obs["tquart"][3:]))
        expected_pose = common.Pose(translation=np.array(expected_obs[:3]), quaternion=np.array(expected_obs[3:]))
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)

    # TODO: collision test still randomly fails
    def test_collision_guard_trpy(self, cfg, gripper_cfg, cam_cfg):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """

        env_ = tf.produce_env_sim_trpy_gripper_camera_cg(
            mjcf_path=TestSimEnvs.mjcf_path,
            urdf_path=TestSimEnvs.urdf_path,
            cfg=cfg,
            gripper_cfg=gripper_cfg,
            cam_cfg=cam_cfg,
            robot_id="0",
        )
        obs, _ = env_.reset()
        # an obvious below ground collision action
        obs["xyzrpy"][2] = -0.2

        collision_action = OrderedDict(
            [
                ("xyzrpy", obs["xyzrpy"]),
                ("gripper", 0),
            ]
        )
        _, _, _, _, info = env_.step(collision_action)
        # This is a scenario of the tcp below the ground which is an obvious collision, however the info.collision
        # property is not populated correctly

        assert info["ik_success"]
        assert info["is_sim_converged"]
        assert info["collision"]


class TestSimEnvsTquart:
    """This class is for testing Tquart sim env functionalities"""

    def test_non_zero_action_tquart(self, cfg):
        """
        Test that a zero action does not change the state significantly in the tquart configuration
        """
        env, _ = tf.produce_env_sim_tquart(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, robot_id="0")
        obs_initial, _ = env.reset()

        # action to be performed
        t = obs_initial["tquart"][:3].tolist()
        q = obs_initial["tquart"][3:].tolist()
        x_pos_change = 0.3
        # updating the x action by 20cm
        t[0] += x_pos_change
        non_zero_action = OrderedDict([("tquart", np.array(t + q, dtype=np.float32))])

        expected_obs = obs_initial["tquart"].copy()
        expected_obs[0] += x_pos_change

        obs, _, _, _, info = env.step(non_zero_action)
        out_pose = common.Pose(translation=np.array(obs["tquart"][:3]), quaternion=np.array(obs["tquart"][3:]))
        expected_pose = common.Pose(translation=np.array(expected_obs[:3]), quaternion=np.array(expected_obs[3:]))
        assert info["ik_success"]
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)

    def test_zero_action_tquart(self, cfg):
        """
        Test that a zero action does not change the state significantly in the tquart configuration
        """
        env, _ = tf.produce_env_sim_tquart(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, robot_id="0")
        obs_initial, info_ = env.reset()

        home_action_vec = obs_initial["tquart"]
        zero_action = OrderedDict([("tquart", home_action_vec)])

        obs, _, _, _, info = env.step(zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]
        out_pose = common.Pose(translation=np.array(obs["tquart"][:3]), quaternion=np.array(obs["tquart"][3:]))
        expected_pose = common.Pose(
            translation=np.array(obs_initial["tquart"][:3]), quaternion=np.array(obs_initial["tquart"][3:])
        )
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)

    def test_relative_zero_action_tquart(self, cfg, gripper_cfg, cam_cfg):
        # env creation
        env = tf.produce_env_sim_tquart_gripper_camera_rel(
            TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, gripper_cfg, cam_cfg, robot_id="0"
        )
        obs_initial, _ = env.reset()
        print(f"{obs_initial = }")
        zero_rel_action = OrderedDict(
            [
                ("tquart", np.array([0, 0, 0, 0, 0, 0, 1.0], dtype=np.float32)),
                ("gripper", 0),
            ]
        )
        obs, _, _, _, info = env.step(zero_rel_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]
        out_pose = common.Pose(translation=np.array(obs["tquart"][:3]), quaternion=np.array(obs["tquart"][3:]))
        expected_pose = common.Pose(
            translation=np.array(obs_initial["tquart"][:3]), quaternion=np.array(obs_initial["tquart"][3:])
        )
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)

    # TODO: collision test still randomly fails
    def test_collision_guard_tquart(self, cfg, gripper_cfg, cam_cfg):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """

        env_ = tf.produce_env_sim_tquart_gripper_camera_cg(
            mjcf_path=TestSimEnvs.mjcf_path,
            urdf_path=TestSimEnvs.urdf_path,
            cfg=cfg,
            gripper_cfg=gripper_cfg,
            cam_cfg=cam_cfg,
            robot_id="0",
        )
        env_.reset()
        # this is a pose with an obvious collision (below the floor)
        act = OrderedDict(
            [
                (
                    "tquart",
                    np.array([0.3, 0, -0.01, 9.23879533e-01, -3.82683432e-01, -7.25288143e-17, -3.00424186e-17]),
                ),
                ("gripper", 0),
            ]
        )
        _, _, _, _, info = env_.step(act)

        assert info["ik_success"]
        assert info["is_sim_converged"]
        assert info["collision"]


class TestSimEnvsJoints:
    """This class is for testing Joints sim env functionalities"""

    def test_zero_action_joints(self, cfg):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        env_, _ = tf.produce_env_sim_joints(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, robot_id="0")
        obs_initial, _ = env_.reset()

        # action to be performed
        zero_action = OrderedDict([("joints", np.array(obs_initial["joints"]))])
        obs, _, _, _, info = env_.step(zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]
        assert np.allclose(obs["joints"], obs_initial["joints"], atol=0.01, rtol=0)

    def test_non_zero_action_joints(self, cfg):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        env_, _ = tf.produce_env_sim_joints(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg, robot_id="0")
        obs_initial, _ = env_.reset()
        # action to be performed
        non_zero_action = OrderedDict(
            [("joints", obs_initial["joints"] + np.array([0.1, 0.1, 0.1, 0.1, -0.1, -0.1, 0.1]))]
        )
        obs, _, _, _, info = env_.step(non_zero_action)
        assert info["ik_success"]
        assert info["is_sim_converged"]

        assert np.allclose(obs["joints"], non_zero_action["joints"], atol=0.01, rtol=0)

    def test_collision_guard_joints(self, cfg, gripper_cfg, cam_cfg):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """

        env_ = tf.produce_env_sim_joints_gripper_camera_cg(
            mjcf_path=TestSimEnvs.mjcf_path,
            urdf_path=TestSimEnvs.urdf_path,
            cfg=cfg,
            gripper_cfg=gripper_cfg,
            cam_cfg=cam_cfg,
            robot_id="0",
        )
        env_.reset()
        # the below action is a test_case where there is an obvious collision regardless of the gripper action
        act = OrderedDict(
            [
                ("joints", np.array([0, 1.78, 0, -1.45, 0, 0, 0], dtype=np.float32)),
                ("gripper", 1),
            ]
        )
        _, _, _, _, info = env_.step(act)
        assert info["ik_success"]
        assert info["is_sim_converged"]
        assert info["collision"]

    def test_relative_zero_action_joints(self, cfg, gripper_cfg, cam_cfg):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """

        env_ = tf.produce_env_sim_joints_gripper_camera_rel(
            mjcf_path=TestSimEnvs.mjcf_path,
            urdf_path=TestSimEnvs.urdf_path,
            cfg=cfg,
            gripper_cfg=gripper_cfg,
            cam_cfg=cam_cfg,
            robot_id="0",
        )
        obs_initial, _ = env_.reset()
        act = OrderedDict(
            [
                ("joints", np.array([0, 0, 0, 0, 0, 0, 0], dtype=np.float32)),
                ("gripper", 0),
            ]
        )
        obs, _, _, _, info = env_.step(act)
        assert info["ik_success"]
        assert info["is_sim_converged"]
        out_pose = common.Pose(translation=np.array(obs["tquart"][:3]), quaternion=np.array(obs["tquart"][3:]))
        expected_pose = common.Pose(
            translation=np.array(obs_initial["tquart"][:3]), quaternion=np.array(obs_initial["tquart"][3:])
        )
        assert out_pose.is_close(expected_pose, 1e-2, 1e-3)
