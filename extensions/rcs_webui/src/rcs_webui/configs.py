"""Catalog of env configs the interface can start from: the default configs of the known envs, config files in
the config directories (``--config-dir``) and the stored configs of previous recordings.

Config files are JSON in the format of ``rcs-webui dump-config``: ``{"creator": "<module>:<EnvCreator>",
"config": <serialized config>}``. A bare serialized config (like ``env_config.json`` of a recording) is accepted
as well: the creator is then the given env or, without one, the generic creator of the config type
(:func:`rcs_webui.envs.infer_creator`).
"""

import json
from pathlib import Path
from typing import Any

from rcs_webui import envs, snapshot

DEFAULT_PREFIX = "default:"
FILE_PREFIX = "file:"
RECORDING_PREFIX = "recording:"


def wrap(creator: str, cfg_json: Any) -> dict[str, Any]:
    return {"creator": creator, "config": cfg_json}


def unwrap(data: Any, env: str | None = None) -> tuple[str, Any]:
    """Returns (creator name, serialized config) of a wrapped or bare config."""
    if isinstance(data, dict) and "creator" in data and "config" in data:
        return str(data["creator"]), data["config"]
    if env is not None:
        return envs.creator_name(envs.load_creator(env)), data
    return envs.infer_creator(snapshot.from_jsonable(data)), data


def normalized(creator: str, cfg_json: Any, frequency: float | None = None) -> dict[str, Any]:
    """Wrapped config after :func:`rcs_webui.envs.normalize_config`, with a summary for the interface."""
    cfg = envs.normalize_config(snapshot.from_jsonable(cfg_json), frequency)
    return {**wrap(creator, snapshot.to_jsonable(cfg)), "summary": envs.describe(cfg, creator)}


def read_file(path: Path, env: str | None = None) -> tuple[str, Any]:
    return unwrap(json.loads(Path(path).read_text()), env)


def default_config(env: str, *, frequency: float, collect: bool, headless: bool) -> dict[str, Any]:
    creator = envs.load_creator(env)
    cfg = envs.prepare_config(
        creator.config(),
        frequency=frequency,
        control_mode=envs.QUEST_CONTROL_MODE if collect else None,
        headless=headless,
    )
    return wrap(envs.creator_name(creator), snapshot.to_jsonable(cfg))


def validate(data: dict[str, Any]) -> Any:
    """Loads the config and its creator, raises with a readable message if that fails. Returns the config."""
    creator, cfg_json = unwrap(data)
    envs.load_creator(creator)
    return snapshot.from_jsonable(cfg_json)


def changed_paths(base: Any, new: Any, prefix: str = "") -> dict[str, Any]:
    """Dotted paths (the syntax of snapshot.apply_overrides) of the values that differ between two serialized
    configs, mapped to the new value. A removed key maps to None."""
    if isinstance(base, dict) and isinstance(new, dict):
        if (
            "fields" in base
            and "fields" in new
            and {k for k in base if k != "fields"} == {k for k in new if k != "fields"}
            and all(base[k] == new[k] for k in base if k != "fields")
        ):
            return changed_paths(base["fields"], new["fields"], prefix)
        if any(k.startswith("__") for k in (*base, *new)):
            # a leaf value (ndarray, enum, pose, ...) or a changed type
            return {} if base == new else {prefix.rstrip("."): new}
        out: dict[str, Any] = {}
        for key in sorted(set(base) | set(new)):
            if key not in new:
                out[f"{prefix}{key}"] = None
            elif key not in base:
                out[f"{prefix}{key}"] = new[key]
            else:
                out.update(changed_paths(base[key], new[key], f"{prefix}{key}."))
        return out
    return {} if base == new else {prefix.rstrip("."): new}


def catalog(config_dirs: list[Path], data_root: Path) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for name in envs.available_envs():
        group = "default configs: hardware" if name in envs.HARDWARE_CREATORS else "default configs: simulation"
        entries.append({"id": DEFAULT_PREFIX + name, "group": group, "label": name})
    for directory in config_dirs:
        if not Path(directory).is_dir():
            continue
        for path in sorted(Path(directory).glob("*.json")):
            entries.append({"id": FILE_PREFIX + str(path.resolve()), "group": str(directory), "label": path.name})
    root = Path(data_root)
    if root.exists():
        for d in sorted(root.iterdir(), reverse=True):
            if (d / snapshot.CONFIG_FILE).exists():
                entries.append({"id": RECORDING_PREFIX + d.name, "group": "previous recordings", "label": d.name})
    return entries


def load_entry(entry_id: str, *, config_dirs: list[Path], data_root: Path, **default_kwargs: Any) -> dict[str, Any]:
    """Returns the wrapped and normalized config of a catalog entry with its summary."""
    wrapped = _load_entry(entry_id, config_dirs=config_dirs, data_root=data_root, **default_kwargs)
    return normalized(wrapped["creator"], wrapped["config"], default_kwargs.get("frequency"))


def _load_entry(entry_id: str, *, config_dirs: list[Path], data_root: Path, **default_kwargs: Any) -> dict[str, Any]:
    if entry_id.startswith(DEFAULT_PREFIX):
        return default_config(entry_id[len(DEFAULT_PREFIX) :], **default_kwargs)
    if entry_id.startswith(FILE_PREFIX):
        path = Path(entry_id[len(FILE_PREFIX) :]).resolve()
        if not any(Path(d).resolve() == path.parent for d in config_dirs):
            msg = "the file is not in a config directory"
            raise ValueError(msg)
        creator, cfg_json = read_file(path)
        return wrap(creator, cfg_json)
    if entry_id.startswith(RECORDING_PREFIX):
        session = (Path(data_root) / entry_id[len(RECORDING_PREFIX) :]).resolve()
        if Path(data_root).resolve() not in session.parents:
            msg = "unknown recording"
            raise ValueError(msg)
        meta = json.loads((session / snapshot.SESSION_FILE).read_text())
        return wrap(meta["creator"], json.loads((session / snapshot.CONFIG_FILE).read_text()))
    msg = f"unknown config {entry_id!r}"
    raise ValueError(msg)
