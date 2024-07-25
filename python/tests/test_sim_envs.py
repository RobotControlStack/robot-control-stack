# import pytest
from collections import OrderedDict

import numpy as np
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSetConfig
from rcsss.envs import typed_factory as tf
from rcsss import sim


class TestSimEnvs:
    mjcf_path = "models/mjcf/fr3_modular/scene.xml"
    urdf_path = "models/fr3/urdf/fr3_from_panda.urdf"

    def test_zero_action(self):
        """
        Test that a zero action does not change the state significantly
        """
        cfg_ = sim.FR3Config()
        cfg_.ik_duration_in_milliseconds = 300
        cfg_.realtime = False
        env, _ = tf.produce_env_sim_trpy(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg_, robot_id="0")
        obs_, info_ = env.reset()
        print(f"initial_obs: {obs_}")
        zero_action = OrderedDict([('xyzrpy', np.array([1,  1,  1,  0,  0, 0], dtype=np.float32))])
        obs, reward, terminated, truncated, info = env.step(zero_action)
        print(f"{obs=}")
        print(f"{obs['tquart'] == obs_['tquart']}")

    def test_non_zero_action_trpy(self):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        cfg_ = sim.FR3Config()
        cfg_.ik_duration_in_milliseconds = 300
        cfg_.realtime = False
        env, _ = tf.produce_env_sim_trpy(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg_, robot_id="0")

        obs_, info_ = env.reset()
        print(f"initial_obs: {obs_}")
        # action to be performed
        # @todo why does a non zero action not change the observed value
        non_zero_action = OrderedDict([('xyzrpy', np.array([1,  1,  1,  0,  0, 0], dtype=np.float32))])
        obs, reward, terminated, truncated, info = env.step(non_zero_action)
        print(f"{obs=}")
        print(f"{obs['tquart'] == obs_['tquart']}")

    def test_non_zero_action_joints(self):
        """
        This is for testing that a certain action leads to the expected change in state
        """
        cfg_ = sim.FR3Config()
        cfg_.ik_duration_in_milliseconds = 300
        cfg_.realtime = False
        env_, _ = tf.produce_env_sim_joints(TestSimEnvs.mjcf_path, TestSimEnvs.urdf_path, cfg_, robot_id="0")
        obs_, info_ = env_.reset()
        print(f"initial_obs: {obs_}")
        # action to be performed
        # @todo why does a non zero action not change the observed value
        non_zero_action = OrderedDict([('joints', np.array([1.2854844,  1.0638355, -1.9478809, -1.8864504,
                                                            -0.09758008, 4.125365,  0.19993608], dtype=np.float32))])

        print(f"initial_obs: {obs_}")
        obs, reward, terminated, truncated, info = env_.step(non_zero_action)
        print(f"{obs=}")
        print(f"{obs['tquart'] == obs_['tquart']}")

    def test_collision_guard(self):
        """
        Check that an obvious collision is detected by the CollisionGuard
        """
        # @todo how to test this functionality?
        cfg_ = sim.FR3Config()
        cfg_.ik_duration_in_milliseconds = 300
        cfg_.realtime = False
        gripper_cfg = sim.FHConfig()

        cameras = {
            "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
            "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
        }
        cam_cfg_ = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=50)

        env_ = tf.produce_env_sim_tquart_gripper_camera_cg_rel(mjcf_path=TestSimEnvs.mjcf_path,
                                                               urdf_path=TestSimEnvs.urdf_path,
                                                               cfg=cfg_,
                                                               gripper_cfg=gripper_cfg,
                                                               cam_cfg=cam_cfg_,
                                                               robot_id='0')
        obs_, info_ = env_.reset()
        print(f"initial_obs: {obs_}")
        for _ in range(2):
            act = env_.action_space.sample()
            # act["tquart"][3:] = [0, 0, 0, 1]
            obs, reward, terminated, truncated, info = env_.step(act)
            print(f"{obs=}")
            if truncated or terminated:
                env_.reset()
