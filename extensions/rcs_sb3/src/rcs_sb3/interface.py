from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

import gymnasium as gym


class AlgorithmInterface(ABC):
    """Framework-neutral interface for trainable RL algorithms."""

    @abstractmethod
    def as_wrapper(self, env: gym.Env) -> gym.Wrapper:
        """Return an RCS wrapper that injects policy actions into the stack."""

    @abstractmethod
    def make_training_env(self, env: gym.Env) -> gym.Env:
        """Return the environment prepared for training in the underlying RL framework."""

    @abstractmethod
    def build(self, env: gym.Env) -> Any:
        """Create and store a model for the wrapped environment."""

    @abstractmethod
    def action(self, observation: Any, deterministic: bool | None = None) -> Any:
        """Generate one environment action from an observation."""

    @abstractmethod
    def learn(self, total_timesteps: int | None = None, **kwargs: Any) -> Any:
        """Train the model."""

    @abstractmethod
    def predict(
        self,
        observation: Any,
        state: Any = None,
        episode_start: Any = None,
        deterministic: bool = False,
    ) -> tuple[Any, Any]:
        """Predict the next action from an observation."""

    @abstractmethod
    def save(self, path: str | Path) -> None:
        """Save the current model."""

    @classmethod
    @abstractmethod
    def load(cls, path: str | Path, env: gym.Env | None = None, **kwargs: Any) -> "AlgorithmInterface":
        """Load a model into an algorithm implementation."""
