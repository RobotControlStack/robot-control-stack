import pytest
from collections import OrderedDict
import numpy as np
import rcsss
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSetConfig
from rcsss.envs import typed_factory as tf
from rcsss import sim


class TestSimEnvs:
    mjcf_path = str(rcsss.scenes["fr3_empty_world"])
    urdf_path = str(rcsss.scenes["lab"].parent / "fr3.urdf")

    def test_model_availability(self):
        assert (
            "lab" in rcsss.scenes
        ), "This pip package was not built with the UTN lab models which is needed for these tests, aborting."

    @pytest.mark.skip(reason="temporarily making it fail due to the TODOs")
    def test_zero_action_trpy(self):
        """
        Test that a zero action does not change the state significantly
        """
        cfg_ = sim.FR3Config()
        cfg_.ik_duration_in_milliseconds = 300
        cfg_.realtime = False
        env, _ = tf.produce_env_sim_trpy(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg_, robot_id="0")
        obs_initial, info_ = env.reset()
        print(f"initial_obs: {obs_initial}")
        print(f"info: {info_}")
        zero_action = OrderedDict([("xyzrpy", np.array([0, 0, 0, 0, 0, 0], dtype=np.float32))])
        print(f"zero_action: {zero_action}")
        # TODO, there seems to be a bug, with two steps of this zero action, the observation changes
        obs, reward, terminated, truncated, info = env.step(zero_action)
        print(f"{obs=}")
        assert np.array_equal(obs["tquart"], obs_initial["tquart"])

    @pytest.mark.skip(reason="temporarily making it fail due to the TODOs")
    def test_non_zero_action_trpy(self):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        # env creation
        cfg_ = sim.FR3Config()
        cfg_.ik_duration_in_milliseconds = 300
        cfg_.realtime = False
        env, _ = tf.produce_env_sim_trpy(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg_, robot_id="0")
        obs_initial, info_ = env.reset()

        print(f"initial_obs: {obs_initial}")
        # action to be performed
        non_zero_action = env.action_space.sample()
        print(f"non_zero_action: {non_zero_action}")
        # TODO, there seems to be a bug, it is required to step two times to see an effect
        obs, reward, terminated, truncated, info = env.step(non_zero_action)
        print(f"{obs=}")
        assert not np.array_equal(obs["tquart"], obs_initial["tquart"])

    @pytest.mark.skip(reason="temporarily making it fail due to the TODOs")
    def test_non_zero_action_joints(self):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        cfg_ = sim.FR3Config()
        cfg_.ik_duration_in_milliseconds = 300
        cfg_.realtime = False
        env_, _ = tf.produce_env_sim_joints(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg_, robot_id="0")
        obs_initial, info_ = env_.reset()
        print(f"initial_obs: {obs_initial}")
        # action to be performed
        non_zero_action = env_.action_space.sample()
        # TODO, there seems to be a bug, it is required to step two times to see an effect
        obs, reward, terminated, truncated, info = env_.step(non_zero_action)
        print(f"{obs=}")
        assert not np.array_equal(obs["tquart"], obs_initial["tquart"])

    @pytest.mark.skip(reason="temporarily making it fail due to the TODOs")
    def test_collision_guard(self):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """
        cfg_ = sim.FR3Config()
        cfg_.ik_duration_in_milliseconds = 300
        cfg_.realtime = False
        gripper_cfg = sim.FHConfig()

        # no camera obs needed for this test
        cameras = {}
        cam_cfg_ = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=50)

        env_ = tf.produce_env_sim_joints_gripper_camera_cg(
            mjcf_path=TestSimEnvs.mjcf_path,
            urdf_path=TestSimEnvs.urdf_path,
            cfg=cfg_,
            gripper_cfg=gripper_cfg,
            cam_cfg=cam_cfg_,
            robot_id="0",
        )
        obs_, info_ = env_.reset()
        print(f"initial_obs: {obs_}")
        act = env_.action_space.sample()
        # the below action is a test_case where there is an obvious collision regardless of the gripper action
        act["joints"] = np.array([0, 1.78, 0, -1.45, 0, 0, 0], dtype=np.float32)
        # TODO, there seems to be a bug, it is required to step two times to see an effect
        obs, reward, terminated, truncated, info = env_.step(act)
        assert info["collision"]
