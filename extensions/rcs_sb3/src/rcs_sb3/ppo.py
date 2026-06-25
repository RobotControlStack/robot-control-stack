from __future__ import annotations

from dataclasses import dataclass

from stable_baselines3 import PPO

from rcs_sb3.sb3 import SB3AlgorithmConfig, StableBaselines3Algorithm


@dataclass
class SB3PPOConfig(SB3AlgorithmConfig):
    """Configuration for Stable-Baselines3 PPO."""


class SB3PPO(StableBaselines3Algorithm):
    """PPO implementation behind the framework-neutral algorithm interface."""

    algorithm_cls = PPO
    config_cls = SB3PPOConfig
