import warnings

import gymnasium as gym
import numpy as np
from rcs.envs.tasks import PickObjSuccessWrapper

import rcs


class _DummyJoint:
    def __init__(self):
        self.qpos = np.array([0.0, 0.0, 0.0])


class _DummyData:
    def __init__(self):
        self._joint = _DummyJoint()
        self.qvel = np.zeros(1)

    def joint(self, _name: str):
        return self._joint


class _DummySim:
    def __init__(self):
        self.data = _DummyData()


class _DummyGripper:
    def get_normalized_width(self):
        return 0.5


class _DummyEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.sim = _DummySim()
        self.gripper = {"robot": _DummyGripper()}

    def get_wrapper_attr(self, name: str):
        return getattr(self, name)

    def step(self, _action):
        obs = {"robot": {"gripper": np.array([0.0]), "tquat": np.zeros(7)}}
        info = {"robot": {"is_grasped": False}}
        return obs, 0.0, False, False, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return {}, {}


def test_pick_obj_success_wrapper_step_avoids_numpy_truth_ambiguity():
    wrapper = PickObjSuccessWrapper(_DummyEnv(), "robot", rcs.common.Pose())

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("error", DeprecationWarning)
        wrapper.step({})

    assert not caught
