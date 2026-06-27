from __future__ import annotations

import copy
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium.spaces import Box, Dict as DictSpace
from gymnasium.spaces import utils as space_utils


class FlattenActionWrapper(gym.ActionWrapper):
    """Expose a flat Box action space while forwarding unflattened actions to RCS."""

    def __init__(self, env: gym.Env):
        super().__init__(env)
        if not isinstance(env.action_space, DictSpace):
            msg = "FlattenActionWrapper expects a gymnasium.spaces.Dict action space."
            raise TypeError(msg)

        flat_space = space_utils.flatten_space(env.action_space)
        if not isinstance(flat_space, Box):
            msg = "Stable-Baselines3 requires flattenable continuous RCS actions."
            raise TypeError(msg)
        self.action_space = Box(
            low=np.asarray(flat_space.low, dtype=np.float32),
            high=np.asarray(flat_space.high, dtype=np.float32),
            dtype=np.float32,
        )

    def action(self, action: np.ndarray) -> dict[str, Any]:
        return space_utils.unflatten(self.env.action_space, np.asarray(action, dtype=self.action_space.dtype))

class TaximSB3ObservationWrapper(gym.ObservationWrapper):
    def __init__(
        self,
        env: gym.Env,
        *,
        left_frame_key: str = "tactile_gripperleft_digit_pad",
        right_frame_key: str = "tactile_gripperright_digit_pad",
    ):
        super().__init__(env)
        self.left_frame_key = left_frame_key
        self.right_frame_key = right_frame_key

        left_space = env.observation_space["frames"][left_frame_key]["rgb"]["data"]
        h, w, c = left_space.shape

        gripper_space = env.observation_space["robot"]["gripper"]

        state_space = copy.deepcopy(env.observation_space)
        del state_space.spaces["frames"]

        self._state_space = state_space
        flat_state_space = space_utils.flatten_space(state_space)

        self.observation_space = gym.spaces.Dict(
            {
                "left_rgb": gym.spaces.Box(0, 255, shape=(c, h, w), dtype=np.uint8),
                "right_rgb": gym.spaces.Box(0, 255, shape=(c, h, w), dtype=np.uint8),
                "state": gripper_space
            }
        )

    def observation(self, obs):
        left_rgb = obs["frames"][self.left_frame_key]["rgb"]["data"]
        right_rgb = obs["frames"][self.right_frame_key]["rgb"]["data"]
        gripper_state = obs['robot']['gripper']
        state_obs = copy.deepcopy(obs)
        del state_obs["frames"]

        return {
            "left_rgb": np.transpose(left_rgb, (2, 0, 1)),
            "right_rgb": np.transpose(right_rgb, (2, 0, 1)),
            "state": gripper_state,
        }

class GripperOnlyActionWrapper(gym.ActionWrapper):
    def __init__(
        self,
        env: gym.Env,
        *,
        robot_key: str = "robot",
        gripper_key: str = "gripper",
        joints_key: str = "joints",
    ):
        super().__init__(env)
        self.robot_key = robot_key
        self.gripper_key = gripper_key
        self.joints_key = joints_key

        robot_action_space = env.action_space[robot_key]
        self._robot_action_space = robot_action_space

        gripper_space = robot_action_space[gripper_key]
        self.action_space = gym.spaces.Box(
            low=np.asarray(gripper_space.low, dtype=np.float32),
            high=np.asarray(gripper_space.high, dtype=np.float32),
            shape=gripper_space.shape,
            dtype=np.float32,
        )

    def action(self, action: np.ndarray) -> dict[str, Any]:
        robot_action = self._robot_action_space.sample()

        for key, space in self._robot_action_space.spaces.items():
            if isinstance(space, gym.spaces.Box):
                robot_action[key] = np.zeros(space.shape, dtype=space.dtype)

        robot_action[self.gripper_key] = np.asarray(action, dtype=np.float32)

        return {
            self.robot_key: robot_action,
        }

class StableBaselines3Wrapper(gym.Wrapper):
    """Prepare an RCS environment for Stable-Baselines3 training.

    RCS environments commonly expose dict action spaces. Stable-Baselines3
    policies do not accept dict actions, so this wrapper flattens them into a
    single continuous Box. Dict observations are flattened by default for
    ``MlpPolicy``. Set ``flatten_observations=False`` for ``MultiInputPolicy``.
    """

    def __init__(
        self,
        env: gym.Env,
        *,
        flatten_actions: bool = True,
        flatten_observations: bool = True,
    ):
        if flatten_actions and isinstance(env.action_space, DictSpace):
            env = FlattenActionWrapper(env)
        elif not flatten_actions and isinstance(env.action_space, DictSpace):
            env = GripperOnlyActionWrapper(env)
        if flatten_observations and isinstance(env.observation_space, DictSpace):
            env = gym.wrappers.FlattenObservation(env)
        elif not flatten_observations and isinstance(env.observation_space, DictSpace):
            env = TaximSB3ObservationWrapper(env)
        super().__init__(env)


class StableBaselines3PolicyWrapper(gym.Wrapper):
    """RCS control wrapper that generates actions with a Stable-Baselines3 model.

    The wrapper keeps the last fully assembled observation returned by lower RCS
    wrappers. On each ``step()``, it predicts a policy action from that stored
    observation, merges the prediction into the incoming action dict, and passes
    the result down toward the robot wrapper.
    """

    ACTION_INFO_KEY = "sb3_action"

    def __init__(
        self,
        env: gym.Env,
        model: Any,
        *,
        deterministic: bool = True,
        flatten_actions: bool = True,
        flatten_observations: bool = True,
        preserve_external_actions: bool = True,
        action_info_key: str | None = ACTION_INFO_KEY,
    ):
        super().__init__(env)
        self.model = model
        self.deterministic = deterministic
        self.flatten_actions = flatten_actions
        self.flatten_observations = flatten_observations
        self.preserve_external_actions = preserve_external_actions
        self.action_info_key = action_info_key
        self._last_observation: Any | None = None
        self._policy_state: Any = None
        self._episode_start = True

    def _model_observation(self, observation: Any) -> Any:
        if self.flatten_observations and isinstance(self.env.observation_space, DictSpace):
            return space_utils.flatten(self.env.observation_space, observation).astype(np.float32)
        return observation

    def _env_action(self, action: Any) -> Any:
        if self.flatten_actions and isinstance(self.env.action_space, DictSpace):
            return space_utils.unflatten(self.env.action_space, np.asarray(action, dtype=np.float32))
        return action

    def policy_action(self, observation: Any | None = None, deterministic: bool | None = None) -> Any:
        observation = self._last_observation if observation is None else observation
        if observation is None:
            msg = "Call reset() before requesting a policy action."
            raise RuntimeError(msg)
        action, self._policy_state = self.model.predict(
            self._model_observation(observation),
            state=self._policy_state,
            episode_start=np.array([self._episode_start]),
            deterministic=self.deterministic if deterministic is None else deterministic,
        )
        self._episode_start = False
        return self._env_action(action)

    def _merge_actions(self, external_action: Any, policy_action: Any) -> Any:
        if not isinstance(policy_action, dict):
            return policy_action
        if external_action is None:
            return policy_action
        if not isinstance(external_action, dict):
            msg = "External action must be a dict when the policy produces dict actions."
            raise TypeError(msg)
        merged_action = copy.deepcopy(policy_action)
        if self.preserve_external_actions:
            merged_action.update(external_action)
        else:
            merged_action = copy.deepcopy(external_action)
            merged_action.update(policy_action)
        return merged_action

    def step(self, action: dict[str, Any] | None = None):
        generated_action = self.policy_action()
        env_action = self._merge_actions(action, generated_action)
        observation, reward, terminated, truncated, info = self.env.step(env_action)
        self._last_observation = observation
        self._episode_start = terminated or truncated
        if self.action_info_key is not None:
            info = dict(info)
            info[self.action_info_key] = copy.deepcopy(generated_action)
        return observation, reward, terminated, truncated, info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[Any, dict[str, Any]]:
        observation, info = self.env.reset(seed=seed, options=options)
        self._last_observation = observation
        self._policy_state = None
        self._episode_start = True
        return observation, info
