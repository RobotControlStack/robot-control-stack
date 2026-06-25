from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar, Type

import gymnasium as gym
from stable_baselines3.common.base_class import BaseAlgorithm

from rcs_sb3.interface import AlgorithmInterface
from rcs_sb3.wrappers import StableBaselines3PolicyWrapper, StableBaselines3Wrapper


@dataclass
class SB3AlgorithmConfig:
    """Configuration shared by Stable-Baselines3 algorithms."""

    policy: str = "MlpPolicy"
    total_timesteps: int = 100_000
    flatten_actions: bool = True
    flatten_observations: bool = True
    deterministic_actions: bool = True
    preserve_external_actions: bool = True
    model_kwargs: dict[str, Any] = field(default_factory=dict)
    learn_kwargs: dict[str, Any] = field(default_factory=dict)


class StableBaselines3Algorithm(AlgorithmInterface):
    """Base class for Stable-Baselines3-backed algorithms."""

    algorithm_cls: ClassVar[Type[BaseAlgorithm]]
    config_cls: ClassVar[type[SB3AlgorithmConfig]] = SB3AlgorithmConfig

    def __init__(self, config: SB3AlgorithmConfig | None = None, model: BaseAlgorithm | None = None):
        self.config = config if config is not None else self.config_cls()
        self.model = model
        self.env: gym.Env | None = None

    def as_wrapper(self, env: gym.Env) -> gym.Wrapper:
        if self.model is None:
            msg = "Call build(env) or load(path, env=env) before creating the RCS policy wrapper."
            raise RuntimeError(msg)
        return StableBaselines3PolicyWrapper(
            env,
            self.model,
            deterministic=self.config.deterministic_actions,
            flatten_actions=self.config.flatten_actions,
            flatten_observations=self.config.flatten_observations,
            preserve_external_actions=self.config.preserve_external_actions,
        )

    def make_training_env(self, env: gym.Env) -> gym.Env:
        return StableBaselines3Wrapper(
            env,
            flatten_actions=self.config.flatten_actions,
            flatten_observations=self.config.flatten_observations,
        )

    def wrap_env(self, env: gym.Env) -> gym.Env:
        """Backward-compatible alias for ``make_training_env``."""
        return self.make_training_env(env)

    def build(self, env: gym.Env) -> BaseAlgorithm:
        self.env = self.make_training_env(env)
        self.model = self.algorithm_cls(self.config.policy, self.env, **self.config.model_kwargs)
        return self.model

    def action(self, observation: Any, deterministic: bool | None = None) -> Any:
        if self.model is None:
            msg = "Call build(env) or load(path, env=env) before action()."
            raise RuntimeError(msg)
        action, _ = self.predict(
            observation,
            deterministic=self.config.deterministic_actions if deterministic is None else deterministic,
        )
        return action

    def learn(self, total_timesteps: int | None = None, **kwargs: Any) -> BaseAlgorithm:
        if self.model is None:
            msg = "Call build(env) or load(path, env=env) before learn()."
            raise RuntimeError(msg)
        learn_kwargs = {**self.config.learn_kwargs, **kwargs}
        return self.model.learn(total_timesteps=total_timesteps or self.config.total_timesteps, **learn_kwargs)

    def predict(
        self,
        observation: Any,
        state: Any = None,
        episode_start: Any = None,
        deterministic: bool = False,
    ) -> tuple[Any, Any]:
        if self.model is None:
            msg = "Call build(env) or load(path, env=env) before predict()."
            raise RuntimeError(msg)
        return self.model.predict(
            observation,
            state=state,
            episode_start=episode_start,
            deterministic=deterministic,
        )

    def save(self, path: str | Path) -> None:
        if self.model is None:
            msg = "Call build(env) or load(path, env=env) before save()."
            raise RuntimeError(msg)
        self.model.save(path)

    @classmethod
    def load(cls, path: str | Path, env: gym.Env | None = None, **kwargs: Any) -> "StableBaselines3Algorithm":
        config = kwargs.pop("config", None) or cls.config_cls()
        wrapped_env = None
        if env is not None:
            wrapped_env = StableBaselines3Wrapper(
                env,
                flatten_actions=config.flatten_actions,
                flatten_observations=config.flatten_observations,
            )
        model = cls.algorithm_cls.load(path, env=wrapped_env, **kwargs)
        instance = cls(config=config, model=model)
        instance.env = wrapped_env
        return instance
