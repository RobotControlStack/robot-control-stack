"""Catalog of env configs the interface can start from: the default configs of the known envs, config files in
the config directories (``--config-dir``) and the stored configs of previous recordings.

Config files are JSON in the format of ``rcs-webui dump-config``: ``{"creator": "<module>:<EnvCreator>",
"config": <serialized config>}``. A bare serialized config (like ``env_config.json`` of a recording) is accepted
as well when the env (creator) is given separately.
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
    if env is None:
        msg = "the config has no creator, give the env it belongs to (--env)"
        raise ValueError(msg)
    return envs.creator_name(envs.load_creator(env)), data


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


def catalog(config_dirs: list[Path], data_root: Path) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for name in envs.available_envs():
        entries.append({"id": DEFAULT_PREFIX + name, "group": "default configs", "label": name})
    for directory in config_dirs:
        for path in sorted(Path(directory).glob("*.json")):
            entries.append({"id": FILE_PREFIX + str(path.resolve()), "group": str(directory), "label": path.name})
    root = Path(data_root)
    if root.exists():
        for d in sorted(root.iterdir(), reverse=True):
            if (d / snapshot.CONFIG_FILE).exists():
                entries.append({"id": RECORDING_PREFIX + d.name, "group": "previous recordings", "label": d.name})
    return entries


def load_entry(entry_id: str, *, config_dirs: list[Path], data_root: Path, **default_kwargs: Any) -> dict[str, Any]:
    """Returns the wrapped config of a catalog entry."""
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
