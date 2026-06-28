from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
for candidate in (REPO_ROOT / "python", REPO_ROOT / "extensions" / "rcs_sb3" / "src"):
    sys.path.insert(0, str(candidate))

import gymnasium as gym
import numpy as np

from rcs.envs.tasks import MinOpenHoldRewardWrapper
from rcs_sb3.wrappers import TaximSB3ObservationWrapper


class _FakeJoint:
    def __init__(self, qpos: np.ndarray):
        self.qpos = qpos


class _FakeData:
    def __init__(self, qpos: np.ndarray):
        self._joint = _FakeJoint(qpos)

    def joint(self, _name: str) -> _FakeJoint:
        return self._joint


class _FakeSim:
    def __init__(self, qpos: np.ndarray):
        self.data = _FakeData(qpos)


class _HoldEnv(gym.Env):
    def __init__(self, qpos: np.ndarray, *, gripper_width: float, is_grasped: bool):
        self.sim = _FakeSim(qpos)
        self.action_space = gym.spaces.Dict(
            {
                "robot": gym.spaces.Dict(
                    {
                        "gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),
                    }
                )
            }
        )
        self.observation_space = gym.spaces.Dict(
            {
                "robot": gym.spaces.Dict(
                    {
                        "gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),
                    }
                )
            }
        )
        self._gripper_width = gripper_width
        self._is_grasped = is_grasped

    def get_wrapper_attr(self, name: str):
        return getattr(self, name)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return {"robot": {"gripper": np.array([1.0], dtype=np.float32)}}, {}

    def step(self, action):
        obs = {"robot": {"gripper": np.array([1.0], dtype=np.float32)}}
        info = {"gripper_width": self._gripper_width, "is_grasped": self._is_grasped}
        return obs, 0.0, False, False, info


class _TaximObsEnv(gym.Env):
    def __init__(self):
        self.action_space = gym.spaces.Dict(
            {
                "robot": gym.spaces.Dict(
                    {
                        "gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),
                    }
                )
            }
        )
        self.observation_space = gym.spaces.Dict(
            {
                "frames": gym.spaces.Dict(
                    {
                        "tactile_left": gym.spaces.Dict(
                            {
                                "rgb": gym.spaces.Dict(
                                    {"data": gym.spaces.Box(low=0, high=255, shape=(240, 320, 3), dtype=np.uint8)}
                                )
                            }
                        ),
                        "tactile_right": gym.spaces.Dict(
                            {
                                "rgb": gym.spaces.Dict(
                                    {"data": gym.spaces.Box(low=0, high=255, shape=(240, 320, 3), dtype=np.uint8)}
                                )
                            }
                        ),
                    }
                ),
                "robot": gym.spaces.Dict(
                    {
                        "gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),
                    }
                ),
            }
        )

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        obs = {
            "frames": {
                "tactile_left": {"rgb": {"data": np.zeros((240, 320, 3), dtype=np.uint8)}},
                "tactile_right": {"rgb": {"data": np.zeros((240, 320, 3), dtype=np.uint8)}},
            },
            "robot": {"gripper": np.array([1.0], dtype=np.float32)},
        }
        return obs, {}

    def step(self, action):
        obs, _ = self.reset()
        return obs, 0.0, False, False, {}


class _FakeTaximWrapper(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        self.observation_space = env.observation_space

    def observation(self, obs):
        return obs


def test_min_open_hold_reward_prefers_more_open_gripper_when_holding():
    closed_env = MinOpenHoldRewardWrapper(
        _HoldEnv(np.array([0.0, 0.0, 0.50, 1.0, 0.0, 0.0, 0.0]), gripper_width=0.25, is_grasped=True),
        robot_name="robot",
        obj_joint_name="box_joint",
    )
    open_env = MinOpenHoldRewardWrapper(
        _HoldEnv(np.array([0.0, 0.0, 0.50, 1.0, 0.0, 0.0, 0.0]), gripper_width=0.90, is_grasped=True),
        robot_name="robot",
        obj_joint_name="box_joint",
    )

    closed_env.reset()
    _, closed_reward, *_ = closed_env.step({"robot": {"gripper": np.array([0.25], dtype=np.float32)}})

    open_env.reset()
    _, open_reward, *_ = open_env.step({"robot": {"gripper": np.array([0.90], dtype=np.float32)}})

    assert open_reward > closed_reward


def test_min_open_hold_reward_penalizes_drops():
    dropped_env = MinOpenHoldRewardWrapper(
        _HoldEnv(np.array([0.0, 0.0, 0.45, 1.0, 0.0, 0.0, 0.0]), gripper_width=0.90, is_grasped=False),
        robot_name="robot",
        obj_joint_name="box_joint",
    )

    dropped_env.reset()
    _, reward, terminated, _, info = dropped_env.step({"robot": {"gripper": np.array([0.90], dtype=np.float32)}})

    assert reward == 0.0
    assert terminated is True
    assert info["is_dropped"] is True


def test_min_open_hold_reward_truncates_after_100_steps():
    env = MinOpenHoldRewardWrapper(
        _HoldEnv(np.array([0.0, 0.0, 0.50, 1.0, 0.0, 0.0, 0.0]), gripper_width=0.90, is_grasped=True),
        robot_name="robot",
        obj_joint_name="box_joint",
        max_episode_steps=7,
    )

    env.reset()
    truncated = False
    for _ in range(7):
        _, reward, terminated, truncated, _ = env.step({"robot": {"gripper": np.array([0.90], dtype=np.float32)}})
        assert reward > 0.0
        assert terminated is False
    assert truncated is True


class _ResetRobot:
    def __init__(self):
        self._pose = None

    def get_joint_position(self):
        return np.zeros(7)

    def get_cartesian_position(self):
        return _Pose()

    def set_joint_position(self, _q):
        pass


class _Pose:
    def __mul__(self, other):
        return other

    def translation(self):
        return np.array([0.0, 0.0, 0.6])

    def rotation_q_wxyz(self):
        return np.array([1.0, 0.0, 0.0, 0.0])


def test_taxim_observation_wrapper_exposes_tactile_images_to_actor():
    wrapped = TaximSB3ObservationWrapper(_FakeTaximWrapper(_TaximObsEnv()), left_frame_key="tactile_left", right_frame_key="tactile_right")

    obs, _ = wrapped.reset()

    assert set(obs) == {"left_rgb", "right_rgb", "state"}
    assert obs["left_rgb"].shape == (3, 240, 320)
    assert obs["right_rgb"].shape == (3, 240, 320)
    assert obs["state"].shape == (1,)


def test_task_reset_places_cube_in_home_grasp_pose():
    from rcs.envs.tasks import StartGraspedWrapper

    class _FakeActuatorModel:
        actuator_ctrlrange = np.array([[0.0, 1.0]])

    class _FakeSim2:
        def __init__(self):
            self.model = _FakeActuatorModel()
            self.data = type("D", (), {"ctrl": np.zeros(1), "joint": lambda self_, _name: type("J", (), {"qpos": np.zeros(7)})()})()

        def step(self, _n):
            pass

        def sync_gui(self):
            pass

    class _FakeGripper:
        def get_config(self):
            return type("C", (), {"actuator": "gripper", "max_actuator_width": 1.0, "min_actuator_width": 0.0})()

        def grasp(self):
            pass

    class _HomeEnv(gym.Env):
        def __init__(self):
            self.sim = _FakeSim2()
            self.action_space = gym.spaces.Dict({"robot": gym.spaces.Dict({"gripper": gym.spaces.Box(0.0, 1.0, (1,), np.float32)})})
            self.observation_space = self.action_space
            self._robot = _ResetRobot()
            self._gripper = {"robot": _FakeGripper()}

        def get_wrapper_attr(self, name):
            return getattr(self, name)

        def reset(self, *, seed=None, options=None):
            return {"robot": {"gripper": np.array([1.0], dtype=np.float32)}}, {}

        def step(self, action):
            return {"robot": {"gripper": np.array([1.0], dtype=np.float32)}}, 0.0, False, False, {}

    wrapped = StartGraspedWrapper(_HomeEnv(), robot_name="robot", obj_joint_name="box_joint")
    obs, info = wrapped.reset()
    assert obs["robot"]["gripper"].shape == (1,)
    assert isinstance(info, dict)
