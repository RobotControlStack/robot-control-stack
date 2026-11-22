"""Which env creators the interface can start and how their default configs are adapted for recording."""

import dataclasses
import importlib
import logging
import pkgutil
from typing import Any

import gymnasium as gym
import rcs.envs.configs  # noqa: F401 registers the rcs/* sim envs
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.scenes import RCSEnvCreator, SimEnvCreator, SimEnvCreatorConfig

logger = logging.getLogger(__name__)

# action space of the Quest operator: tquat relative to the origin captured when the trigger is pressed
QUEST_CONTROL_MODE = (ControlMode.CARTESIAN_TQuat, RelativeTo.CONFIGURED_ORIGIN)

# hardware creators of the FR3 extension, only available if rcs_fr3 is installed
HARDWARE_CREATORS = {
    "fr3": "rcs_webui.hw_envs:SingleFR3Env",
    "fr3_duo": "rcs_fr3.configs:FrankaDuoEnv",
    "fr3_dual": "rcs_fr3.configs:DefaultFR3DualMultiHardwareEnv",
    "droid": "rcs_fr3.configs:DROIDEnv",
}


# packages whose submodules register more RCS envs on import (each task module calls gym.register),
# imported if installed
DEFAULT_ENV_PACKAGES = ["duobench.tasks"]

_imported_packages: dict[str, str | None] = {}


def import_env_packages(packages: list[str]) -> dict[str, str | None]:
    """Imports every submodule of the given packages so that their envs are registered.

    Returns per package None if it was imported or the reason why not (not installed, import error).
    Each package is only imported once per process.
    """
    for package in packages:
        if package in _imported_packages:
            continue
        try:
            pkg = importlib.import_module(package)
        except ImportError as e:
            _imported_packages[package] = f"not installed ({e})"
            continue
        errors = []
        for info in pkgutil.iter_modules(getattr(pkg, "__path__", []), prefix=f"{package}."):
            try:
                importlib.import_module(info.name)
            except Exception as e:  # a broken task must not hide the others
                logger.warning("importing %s failed: %s", info.name, e)
                errors.append(f"{info.name}: {type(e).__name__}: {e}")
        _imported_packages[package] = "; ".join(errors) or None
    return {p: _imported_packages[p] for p in packages}


def _is_rcs_env(name: str) -> bool:
    entry_point = gym.registry[name].entry_point
    if isinstance(entry_point, RCSEnvCreator):
        return True
    return isinstance(entry_point, str) and name.startswith("rcs/")


def available_envs() -> list[str]:
    """Names that can be passed to :func:`load_creator`: gym ids registered with an RCSEnvCreator (rcs/*, and
    e.g. duobench/* after :func:`import_env_packages`) and the hardware creators."""
    names = [name for name in gym.registry if _is_rcs_env(name)]
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


class ConfigSimEnvCreator(SimEnvCreator):
    """Builds any sim env from a complete config (scene, robots, cameras and the task in ``cfg.task_cfg``).
    Used for config files without a creator; it has no default config of its own."""

    def config(self) -> SimEnvCreatorConfig:
        msg = "ConfigSimEnvCreator only starts complete configs, it has no default config"
        raise NotImplementedError(msg)


def infer_creator(cfg: Any) -> str:
    """Creator for a config without one (e.g. a bare ``env_config.json``): the generic, config driven creator
    that builds envs of this config type (sim: ``SimEnvCreator``, the task comes from ``cfg.task_cfg``)."""
    if is_sim_config(cfg):
        return f"{__name__}:ConfigSimEnvCreator"
    for name in HARDWARE_CREATORS:
        try:
            creator = load_creator(name)
            default_type = type(creator.config())
        except Exception:
            continue
        if type(cfg) is default_type:
            owner = next(c for c in type(creator).__mro__ if "create_env" in vars(c))
            return f"{owner.__module__}:{owner.__qualname__}"
    msg = f"no creator known for a {type(cfg).__qualname__}, use a config file with a creator"
    raise ValueError(msg)


def _robot_platform_hardware(cfg: Any) -> Any:
    """Hardware robot configs carry the C++ default robot_platform=SIMULATION, which is never read on hardware
    (the env class decides) but is confusing in the stored config, so it is set to HARDWARE."""
    if is_sim_config(cfg):
        return cfg
    from rcs._core.common import RobotPlatform

    for robot_cfg in _robot_cfgs(cfg):
        if hasattr(robot_cfg, "robot_platform"):
            robot_cfg.robot_platform = RobotPlatform.HARDWARE
    return cfg


def normalize_config(cfg: Any, frequency: float | None = None) -> Any:
    """Applied to every config the interface shows or starts, whatever its source: the control rate of the
    interface (``frequency``, see :func:`apply_frequency`) replaces the one in the config."""
    cfg = _robot_platform_hardware(cfg)
    return cfg if frequency is None else apply_frequency(cfg, frequency)


def describe(cfg: Any, creator: str) -> dict[str, Any]:
    """Short summary of a config for the interface."""
    robots = list(cfg.robot_cfgs) if hasattr(cfg, "robot_cfgs") else (["robot"] if hasattr(cfg, "robot_cfg") else [])
    cameras = list(getattr(cfg, "camera_cfgs", None) or {})
    if not is_sim_config(cfg) and cameras:
        # hardware: camera_cfgs are camera creators (e.g. "zed") with the cameras inside
        cameras = [name for c in cfg.camera_cfgs.values() for name in (getattr(c, "camera_cfgs", None) or {c: None})]
    frequency = cfg.sim_cfg.frequency if is_sim_config(cfg) else getattr(cfg, "frequency", None)
    control_mode = getattr(cfg, "control_mode", None)
    relative_to = getattr(cfg, "relative_to", None)
    task = getattr(getattr(cfg, "task_cfg", None), "task_id", None)
    return {
        "creator": creator,
        "platform": "simulation" if is_sim_config(cfg) else "hardware",
        "config_type": type(cfg).__qualname__,
        "robots": robots,
        "cameras": [str(c) for c in cameras],
        "frequency": frequency,
        "control_mode": getattr(control_mode, "name", None),
        "relative_to": getattr(relative_to, "name", None),
        "task": task,
    }


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
    - asynchronous control so that the controller thread runs (sim: in realtime), at ``frequency``
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
        cfg.headless = headless
    else:
        for robot_cfg in _robot_cfgs(cfg):
            if hasattr(robot_cfg, "async_control"):
                robot_cfg.async_control = True
    return apply_frequency(cfg, frequency)


def apply_frequency(cfg: Any, frequency: float) -> Any:
    """Sets the control rate of the env, the "control rate" of the interface: the rate of the env steps and of the
    goals for the controller, whose interpolation window is one period (sim: ``sim_cfg.frequency``, hardware: the
    env's ``frequency`` and the robots' ``policy_rate``)."""
    if is_sim_config(cfg):
        cfg.sim_cfg.frequency = frequency
        return cfg
    if dataclasses.is_dataclass(cfg) and hasattr(cfg, "frequency"):
        cfg.frequency = frequency
    for robot_cfg in _robot_cfgs(cfg):
        if hasattr(robot_cfg, "policy_rate"):
            robot_cfg.policy_rate = int(round(frequency))
    return cfg
