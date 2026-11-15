"""Which env creators the interface can start and how their default configs are adapted for recording."""

import dataclasses
import importlib
import logging
from typing import Any

import gymnasium as gym
import rcs.envs.configs  # noqa: F401 registers the rcs/* sim envs
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.scenes import RCSEnvCreator, SimEnvCreatorConfig

logger = logging.getLogger(__name__)

# hardware creators of the FR3 extension, only available if rcs_fr3 is installed
HARDWARE_CREATORS = {
    "fr3": "rcs_webui.hw_envs:SingleFR3Env",
    "fr3_duo": "rcs_fr3.configs:FrankaDuoEnv",
    "fr3_dual": "rcs_fr3.configs:DefaultFR3DualMultiHardwareEnv",
    "droid": "rcs_fr3.configs:DROIDEnv",
}


def available_envs() -> list[str]:
    """Names that can be passed to :func:`load_creator`: registered rcs gym ids and the hardware creators."""
    names = [name for name in gym.registry if name.startswith("rcs/")]
    try:
        importlib.import_module("rcs_fr3")
        names += list(HARDWARE_CREATORS)
    except ImportError:
        pass
    return sorted(names)


def load_creator(name: str) -> RCSEnvCreator:
    """Returns an env creator by gym id (``rcs/fr3``), hardware short name (``fr3_duo``) or import path
    (``my_package.my_module:MyCreator``)."""
    if name in HARDWARE_CREATORS:
        name = HARDWARE_CREATORS[name]
    if ":" in name:
        module_name, _, attr = name.partition(":")
        creator = getattr(importlib.import_module(module_name), attr)
        return creator() if isinstance(creator, type) else creator
    entry_point = gym.spec(name).entry_point
    if isinstance(entry_point, str):
        module_name, _, attr = entry_point.partition(":")
        entry_point = getattr(importlib.import_module(module_name), attr)
    if not isinstance(entry_point, RCSEnvCreator):
        msg = f"{name} is not registered with an RCSEnvCreator entry point"
        raise TypeError(msg)
    return entry_point


def creator_name(creator: RCSEnvCreator) -> str:
    return f"{type(creator).__module__}:{type(creator).__qualname__}"


def is_sim_config(cfg: Any) -> bool:
    return isinstance(cfg, SimEnvCreatorConfig)


def _robot_cfgs(cfg: Any) -> list[Any]:
    if hasattr(cfg, "robot_cfgs"):
        return list(cfg.robot_cfgs.values())
    if hasattr(cfg, "robot_cfg"):
        return [cfg.robot_cfg]
    return []


def prepare_config(
    cfg: Any,
    *,
    frequency: float,
    control_mode: tuple[ControlMode, RelativeTo] | None,
    headless: bool,
) -> Any:
    """Adapts a default creator config for recording with the interface.

    Applied only to default configs (a config loaded from a previous recording is used as stored):

    - the forwarding of the controller rate robot states is switched on (``wrapper_cfg.record_control_states``)
    - the env runs at ``frequency`` (sim: async control in realtime, hardware: rate limited steps and the
      controller interpolation window), asynchronous control so that the controller thread runs
    - the action space matches the operator (e.g. Quest: tquat relative to the configured origin)
    """
    if hasattr(cfg, "wrapper_cfg"):
        cfg.wrapper_cfg.record_control_states = True
    if control_mode is not None:
        cfg.control_mode = control_mode[0]
        cfg.relative_to = control_mode[1]
        if cfg.max_relative_movement is None and control_mode[1] != RelativeTo.NONE:
            import numpy as np

            cfg.max_relative_movement = 0.5 if control_mode[0] == ControlMode.JOINTS else (0.5, np.deg2rad(90))
    if is_sim_config(cfg):
        cfg.sim_cfg.async_control = True
        cfg.sim_cfg.realtime = True
        cfg.sim_cfg.frequency = frequency
        cfg.headless = headless
    else:
        if dataclasses.is_dataclass(cfg) and hasattr(cfg, "frequency"):
            cfg.frequency = frequency
        for robot_cfg in _robot_cfgs(cfg):
            for attr, value in (("async_control", True), ("policy_rate", int(round(frequency)))):
                if hasattr(robot_cfg, attr):
                    setattr(robot_cfg, attr, value)
    return cfg
