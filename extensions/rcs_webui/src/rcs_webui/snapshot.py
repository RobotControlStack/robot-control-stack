"""Reproducibility snapshot of an env start: the complete env config as JSON, git commit and git diff.

The env creator configs are dataclasses that contain pybind11 objects (robot, gripper, camera and sim configs,
poses, enums). They are serialized into plain JSON with type tags so that a new env can be created from the
stored file again, see :func:`config_to_json` and :func:`config_from_json`.
"""

import dataclasses
import datetime
import enum
import importlib
import inspect
import json
import logging
import re
import subprocess
import typing
from pathlib import Path
from typing import Any

import numpy as np

import rcs
from rcs import common

logger = logging.getLogger(__name__)

CONFIG_FILE = "env_config.json"
SESSION_FILE = "session.json"
GIT_FILE = "git.json"
DIFF_SUFFIX = ".diff"
DATA_DIR = "data"

# fields that only exist at runtime and must not be stored
_SKIP_FIELDS = {"_original_cfg"}


def _qualname(obj_type: type) -> str:
    return f"{obj_type.__module__}:{obj_type.__qualname__}"


def _import(qualname: str) -> Any:
    module_name, _, attr_path = qualname.partition(":")
    obj: Any = importlib.import_module(module_name)
    for attr in attr_path.split("."):
        obj = getattr(obj, attr)
    return obj


def _is_pybind_enum(value: Any) -> bool:
    return hasattr(type(value), "__members__") and hasattr(value, "name") and hasattr(value, "value")


def _readwrite_properties(obj_type: type) -> list[str]:
    names = []
    for name in dir(obj_type):
        attr = getattr(obj_type, name, None)
        if isinstance(attr, property) and attr.fset is not None and not name.startswith("_"):
            names.append(name)
    return names


def to_jsonable(value: Any) -> Any:
    """Converts a config (dataclasses, pybind11 configs, poses, enums, numpy arrays) into JSON compatible data."""
    if isinstance(value, str):
        prefix = rcs.RCS_PREFIX.rstrip("/") + "/"
        if value.startswith(prefix):
            # asset paths of RCS (robot models etc.) are stored relative to the asset prefix of the machine
            return {"__rcs_prefix__": value[len(prefix) :]}
        return value
    if value is None or isinstance(value, bool | int | float):
        return value
    if isinstance(value, enum.Enum):
        return {"__enum__": _qualname(type(value)), "name": value.name}
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return {"__ndarray__": value.tolist(), "dtype": str(value.dtype)}
    if isinstance(value, common.Pose):
        return {"__pose__": value.pose_matrix().tolist()}
    if isinstance(value, type):
        return {"__type__": _qualname(value)}
    if dataclasses.is_dataclass(value):
        return {
            "__dataclass__": _qualname(type(value)),
            "fields": {
                f.name: to_jsonable(getattr(value, f.name))
                for f in dataclasses.fields(value)
                if f.name not in _SKIP_FIELDS
            },
        }
    if isinstance(value, dict):
        if all(isinstance(k, str) for k in value):
            return {k: to_jsonable(v) for k, v in value.items()}
        return {"__items__": [[to_jsonable(k), to_jsonable(v)] for k, v in value.items()]}
    if isinstance(value, tuple):
        return {"__tuple__": [to_jsonable(v) for v in value]}
    if isinstance(value, list):
        return [to_jsonable(v) for v in value]
    if _is_pybind_enum(value):
        return {"__enum__": _qualname(type(value)), "name": value.name}
    obj_type = type(value)
    props = _readwrite_properties(obj_type)
    # python subclasses of pybind11 configs (e.g. RobotiQ2F85GripperConfig) keep their own fields in __dict__
    props += [k for k in getattr(value, "__dict__", {}) if not k.startswith("_") and k not in props]
    if props:
        return {
            "__object__": _qualname(obj_type),
            "fields": {name: to_jsonable(getattr(value, name)) for name in props},
        }
    if hasattr(value, "id") and isinstance(value.id, str):
        # e.g. RobotType / GripperType which are constructed from their id
        return {"__object__": _qualname(obj_type), "init": [value.id]}
    msg = f"cannot serialize config value of type {obj_type}"
    raise TypeError(msg)


def _init_kwargs(cls: type, fields: dict[str, Any]) -> dict[str, Any]:
    """Keyword arguments of a pybind11 constructor that are present in the stored fields."""
    try:
        params = inspect.signature(cls).parameters
        if not any(p.kind == p.VAR_POSITIONAL for p in params.values()):
            return {k: v for k, v in fields.items() if k in params}
    except (TypeError, ValueError):
        pass
    doc = getattr(getattr(cls, "__init__", None), "__doc__", None) or ""
    names = set(re.findall(r"(\w+):", doc))
    return {k: v for k, v in fields.items() if k in names}


def from_jsonable(value: Any) -> Any:
    """Inverse of :func:`to_jsonable`."""
    if isinstance(value, list):
        return [from_jsonable(v) for v in value]
    if not isinstance(value, dict):
        return value
    if "__rcs_prefix__" in value:
        return str(Path(rcs.RCS_PREFIX) / value["__rcs_prefix__"])
    if "__enum__" in value:
        enum_type = _import(value["__enum__"])
        return enum_type[value["name"]] if isinstance(enum_type, enum.EnumMeta) else getattr(enum_type, value["name"])
    if "__ndarray__" in value:
        return np.array(value["__ndarray__"], dtype=value["dtype"])
    if "__pose__" in value:
        return common.Pose(pose_matrix=np.array(value["__pose__"]))
    if "__type__" in value:
        return _import(value["__type__"])
    if "__tuple__" in value:
        return tuple(from_jsonable(v) for v in value["__tuple__"])
    if "__items__" in value:
        return {from_jsonable(k): from_jsonable(v) for k, v in value["__items__"]}
    if "__dataclass__" in value:
        cls = _import(value["__dataclass__"])
        fields = {k: from_jsonable(v) for k, v in value["fields"].items()}
        known = {f.name for f in dataclasses.fields(cls) if f.init}
        return cls(**{k: v for k, v in fields.items() if k in known})
    if "__object__" in value:
        cls = _import(value["__object__"])
        if "init" in value:
            return cls(*value["init"])
        fields = {k: from_jsonable(v) for k, v in value["fields"].items()}
        try:
            obj = cls()
        except TypeError:
            obj = cls(**_init_kwargs(cls, fields))
        for name, field_value in fields.items():
            try:
                setattr(obj, name, field_value)
            except (AttributeError, TypeError) as e:
                logger.warning("could not restore %s.%s: %s", cls.__name__, name, e)
        return obj
    return {k: from_jsonable(v) for k, v in value.items()}


def config_to_json(cfg: Any) -> str:
    return json.dumps(to_jsonable(cfg), indent=2)


def config_from_json(text: str) -> Any:
    return from_jsonable(json.loads(text))


def apply_overrides(cfg_json: Any, overrides: dict[str, Any]) -> Any:
    """Sets values in a serialized config, keys are dotted paths, e.g. ``{"sim_cfg.frequency": 15}``.

    Path elements walk through dict keys and the ``fields`` of dataclasses and pybind11 objects. Values are
    given in the serialized form (plain JSON for numbers, strings, lists, booleans).
    """
    for path, new_value in overrides.items():
        node = cfg_json
        parts = path.split(".")
        for part in parts[:-1]:
            node = _child(node, part, path)
        last = parts[-1]
        container = node["fields"] if isinstance(node, dict) and "fields" in node else node
        if not isinstance(container, dict) or (last not in container and "fields" in node):
            msg = f"unknown config key {path!r}"
            raise KeyError(msg)
        old = container.get(last)
        value = new_value
        if isinstance(old, dict) and "__ndarray__" in old and isinstance(new_value, list):
            value = {"__ndarray__": new_value, "dtype": old["dtype"]}
        elif isinstance(old, dict) and "__enum__" in old and isinstance(new_value, str):
            value = {"__enum__": old["__enum__"], "name": new_value}
        container[last] = value
    return cfg_json


def _child(node: Any, part: str, path: str) -> Any:
    if isinstance(node, dict) and "fields" in node:
        node = node["fields"]
    if isinstance(node, dict) and part in node:
        return node[part]
    msg = f"unknown config key {path!r} (at {part!r})"
    raise KeyError(msg)


def _git(args: list[str], cwd: Path) -> str | None:
    try:
        out = subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True, check=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout


def git_state(path: Path) -> dict[str, Any] | None:
    """Commit, branch and diff (incl. untracked file names) of the git repository that contains path."""
    path = path if path.is_dir() else path.parent
    root = _git(["rev-parse", "--show-toplevel"], path)
    if root is None:
        return None
    root_path = Path(root.strip())
    return {
        "root": str(root_path),
        "commit": (_git(["rev-parse", "HEAD"], root_path) or "").strip(),
        "branch": (_git(["rev-parse", "--abbrev-ref", "HEAD"], root_path) or "").strip(),
        "remote": (_git(["remote", "get-url", "origin"], root_path) or "").strip(),
        "untracked": (_git(["ls-files", "--others", "--exclude-standard"], root_path) or "").splitlines(),
        "diff": _git(["diff", "HEAD"], root_path) or "",
    }


def config_modules(cfg_json: Any) -> set[str]:
    """Modules of the classes and enums in a serialized config (``__dataclass__``, ``__object__``, ``__enum__``)."""
    found: set[str] = set()
    if isinstance(cfg_json, dict):
        for key in ("__dataclass__", "__object__", "__enum__"):
            if isinstance(cfg_json.get(key), str):
                found.add(cfg_json[key].partition(":")[0])
        for value in cfg_json.values():
            found |= config_modules(value)
    elif isinstance(cfg_json, list):
        for value in cfg_json:
            found |= config_modules(value)
    return found


def _package_version(top_level: str) -> str | None:
    try:
        from importlib.metadata import packages_distributions, version

        dists = packages_distributions().get(top_level) or [top_level]
        return version(dists[0])
    except Exception:
        return None


def code_locations(modules: set[str] | None = None) -> dict[str, Path]:
    """Source locations whose git state is logged: RCS itself, its extensions, the packages of ``modules`` (e.g.
    the env creator and the config classes: a task package like duobench) and the current working directory."""
    import rcs

    locations = {"rcs": Path(rcs.__file__).resolve()}
    for name in ("rcs_fr3", "rcs_webui", "rcs_robotiq2f85", "rcs_realsense", "rcs_zed"):
        try:
            module = importlib.import_module(name)
        except Exception:
            continue
        if getattr(module, "__file__", None):
            locations[name] = Path(typing.cast(str, module.__file__)).resolve()
    for module_name in sorted(modules or ()):
        top = module_name.partition(".")[0]
        if not top or top in locations or top == "builtins":
            continue
        try:
            module = importlib.import_module(module_name)
        except Exception:
            continue
        if getattr(module, "__file__", None):
            locations[top] = Path(typing.cast(str, module.__file__)).resolve()
    locations["cwd"] = Path.cwd()
    return locations


def write_snapshot(session_dir: Path, cfg: Any, meta: dict[str, Any]) -> None:
    """Writes env_config.json, session.json, git.json and one <repo>.diff per git repository."""
    session_dir.mkdir(parents=True, exist_ok=True)
    (session_dir / CONFIG_FILE).write_text(config_to_json(cfg))
    repos: dict[str, dict[str, Any]] = {}
    modules = config_modules(to_jsonable(cfg))
    if meta.get("creator"):
        modules.add(str(meta["creator"]).partition(":")[0])
    for name, location in code_locations(modules).items():
        state = git_state(location)
        if state is None:
            # not in a git checkout (e.g. installed from PyPI): at least the installed version
            if name != "cwd" and (version := _package_version(name)) is not None:
                repos[name] = {"root": None, "path": str(location), "version": version}
            continue
        if any(r["root"] == state["root"] for r in repos.values()):
            continue
        state["version"] = _package_version(name) if name != "cwd" else None
        diff = state.pop("diff")
        diff_name = f"{name}{DIFF_SUFFIX}"
        (session_dir / diff_name).write_text(diff)
        state["diff_file"] = diff_name
        state["dirty"] = bool(diff.strip()) or bool(state["untracked"])
        repos[name] = state
    (session_dir / GIT_FILE).write_text(json.dumps(repos, indent=2))
    meta = {"created": datetime.datetime.now().isoformat(timespec="seconds"), **meta}
    (session_dir / SESSION_FILE).write_text(json.dumps(meta, indent=2, default=str))


def load_session_config(session_dir: Path) -> tuple[Any, dict[str, Any]]:
    """Loads the env config and the session meta data of a previous recording."""
    cfg = config_from_json((session_dir / CONFIG_FILE).read_text())
    meta = json.loads((session_dir / SESSION_FILE).read_text()) if (session_dir / SESSION_FILE).exists() else {}
    return cfg, meta
