import gymnasium as gym
import numpy as np

from rcs_sb3 import SB3PPO, SB3PPOConfig, StableBaselines3PolicyWrapper, StableBaselines3Wrapper


class DummyRCSEnv(gym.Env):
    def __init__(self):
        self.action_space = gym.spaces.Dict(
            {
                "joints": gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float64),
                "gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),
            }
        )
        self.observation_space = gym.spaces.Dict(
            {
                "joints": gym.spaces.Box(low=-10.0, high=10.0, shape=(2,), dtype=np.float64),
                "gripper": gym.spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),
            }
        )
        self.last_action = None

    def step(self, action):
        self.last_action = action
        return {"joints": np.zeros(2), "gripper": np.ones(1)}, 0.0, False, False, {}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return {"joints": np.zeros(2), "gripper": np.ones(1)}, {}


class FakeSB3Model:
    def predict(self, observation, state=None, episode_start=None, deterministic=False):
        assert observation.shape == (3,)
        return np.array([0.25, -0.25, 1.0], dtype=np.float32), state


def test_stable_baselines3_wrapper_flattens_dict_action_and_observation_spaces():
    env = DummyRCSEnv()
    wrapped = StableBaselines3Wrapper(env)

    assert isinstance(wrapped.action_space, gym.spaces.Box)
    assert isinstance(wrapped.observation_space, gym.spaces.Box)

    wrapped.step(np.array([0.5, -0.5, 1.0], dtype=np.float32))

    assert set(env.last_action.keys()) == {"joints", "gripper"}
    np.testing.assert_allclose(env.last_action["joints"], np.array([0.5, -0.5]))
    np.testing.assert_allclose(env.last_action["gripper"], np.array([1.0]))


def test_policy_wrapper_generates_action_from_last_observation():
    env = DummyRCSEnv()
    wrapped = StableBaselines3PolicyWrapper(env, FakeSB3Model())

    wrapped.reset()
    _, _, _, _, info = wrapped.step()

    np.testing.assert_allclose(env.last_action["joints"], np.array([0.25, -0.25]))
    np.testing.assert_allclose(env.last_action["gripper"], np.array([1.0]))
    np.testing.assert_allclose(info["sb3_action"]["joints"], np.array([0.25, -0.25]))


def test_ppo_build_uses_wrapped_env_without_training():
    env = DummyRCSEnv()
    trainer = SB3PPO(SB3PPOConfig(model_kwargs={"n_steps": 2, "batch_size": 2}))
    model = trainer.build(env)

    assert model.env is not None
