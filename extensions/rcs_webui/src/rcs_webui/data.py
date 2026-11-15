"""Read access to recorded sessions for the viewer and in place relabelling / deletion of episodes.

The StorageWrapper writes every flush into its own parquet file and flushes at every reset, so a file
normally holds rows of a single episode. Relabel and delete rewrite every file that contains the episode
(atomically via a temporary file and ``os.replace``), so they also work on consolidated datasets.
"""

import json
import os
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.dataset as ds
import pyarrow.parquet as pq
from rcs_webui import snapshot

EPISODE_COLUMNS = ["uuid", "step", "success", "timestamp", "instruction"]
# robot state signals of the observation that the viewer plots per step
OBS_SIGNALS = ["joints", "gripper", "tquat"]
# control loop signals that the viewer plots at controller rate
CONTROL_SIGNALS = ["q", "dq", "tau_J", "dtau_J", "tau_ext_hat_filtered", "O_F_ext_hat_K"]
MAX_POINTS = 4000


class DataError(Exception):
    pass


def session_path(data_root: Path, name: str) -> Path:
    path = (Path(data_root) / name).resolve()
    if Path(data_root).resolve() not in path.parents or not (path / snapshot.CONFIG_FILE).exists():
        msg = f"unknown session {name!r}"
        raise DataError(msg)
    return path


def _data_dir(session_dir: Path) -> Path:
    return session_dir / snapshot.DATA_DIR


def _files(session_dir: Path) -> list[Path]:
    data_dir = _data_dir(session_dir)
    if not data_dir.exists():
        return []
    return sorted(p for p in data_dir.rglob("*.parquet") if not p.name.startswith("."))


def _dataset(session_dir: Path) -> ds.Dataset | None:
    files = _files(session_dir)
    if not files:
        return None
    return ds.dataset([str(f) for f in files], format="parquet")


def list_sessions(data_root: Path) -> list[dict[str, Any]]:
    out = []
    root = Path(data_root)
    if not root.exists():
        return out
    for d in sorted(root.iterdir(), reverse=True):
        if not (d / snapshot.CONFIG_FILE).exists():
            continue
        meta = json.loads((d / snapshot.SESSION_FILE).read_text()) if (d / snapshot.SESSION_FILE).exists() else {}
        out.append({"name": d.name, **meta})
    return out


def list_episodes(session_dir: Path) -> list[dict[str, Any]]:
    dataset = _dataset(session_dir)
    if dataset is None:
        return []
    names = set(dataset.schema.names)
    columns = [c for c in [*EPISODE_COLUMNS, "truncated"] if c in names]
    table = dataset.to_table(columns=columns)
    episodes: dict[str, dict[str, Any]] = {}
    for row in table.to_pylist():
        ep = episodes.setdefault(
            row["uuid"],
            {"uuid": row["uuid"], "steps": 0, "start": row["timestamp"], "end": row["timestamp"], "last_step": -1},
        )
        ep["steps"] += 1
        ep["start"] = min(ep["start"], row["timestamp"])
        ep["end"] = max(ep["end"], row["timestamp"])
        if row["step"] > ep["last_step"]:
            ep["last_step"] = row["step"]
            ep["success"] = bool(row["success"])
            ep["truncated"] = bool(row.get("truncated", False))
            ep["instruction"] = row["instruction"]
    result = sorted(episodes.values(), key=lambda e: e["start"])
    for ep in result:
        ep["duration"] = ep["end"] - ep["start"]
        del ep["last_step"]
    return result


def _field(path: str) -> ds.Expression:
    return ds.field(*path.split("."))


def _struct_paths(schema: pa.Schema | pa.StructType, prefix: str = "") -> dict[str, pa.DataType]:
    out: dict[str, pa.DataType] = {}
    for f in schema:
        name = f"{prefix}{f.name}"
        if pa.types.is_struct(f.type):
            out.update(_struct_paths(f.type, name + "."))
        else:
            out[name] = f.type
    return out


def _episode_table(session_dir: Path, uuid: str, columns: dict[str, ds.Expression]) -> pa.Table:
    dataset = _dataset(session_dir)
    if dataset is None:
        msg = "no data in session"
        raise DataError(msg)
    table = dataset.to_table(columns={"step": ds.field("step"), **columns}, filter=ds.field("uuid") == uuid)
    if table.num_rows == 0:
        msg = f"unknown episode {uuid!r}"
        raise DataError(msg)
    return table.sort_by("step")


def _downsample(n: int) -> np.ndarray:
    if n <= MAX_POINTS:
        return np.arange(n)
    return np.linspace(0, n - 1, MAX_POINTS).astype(int)


def episode_signals(session_dir: Path, uuid: str) -> dict[str, Any]:
    """Per step signals (observation, actions) and controller rate signals of one episode for plotting.

    Returns times relative to the first step, signals as lists of rows, cameras and labels.
    """
    dataset = _dataset(session_dir)
    if dataset is None:
        msg = "no data in session"
        raise DataError(msg)
    paths = _struct_paths(dataset.schema)
    robots = sorted({p.split(".")[1] for p in paths if p.startswith("obs.") and p.endswith(".joints")})
    cameras = sorted({p.split(".")[2] for p in paths if p.startswith("obs.frames.") and p.endswith(".rgb.data")})
    wanted: dict[str, ds.Expression] = {"timestamp": ds.field("timestamp"), "success": ds.field("success")}
    for extra in ("truncated", "instruction"):
        if extra in paths:
            wanted[extra] = ds.field(extra)
    for robot in robots:
        for sig in OBS_SIGNALS:
            if f"obs.{robot}.{sig}" in paths:
                wanted[f"obs.{robot}.{sig}"] = _field(f"obs.{robot}.{sig}")
        for path in paths:
            if path.startswith(f"env_action.{robot}."):
                wanted[path] = _field(path)
        if f"info.{robot}.robot_error" in paths:
            wanted[f"info.{robot}.robot_error"] = _field(f"info.{robot}.robot_error")
        base = f"info.{robot}.control_states"
        if f"{base}.timestamp" in paths:
            for sig in ["timestamp", *CONTROL_SIGNALS]:
                for suffix in ("", "_shape"):
                    if f"{base}.{sig}{suffix}" in paths:
                        wanted[f"{base}.{sig}{suffix}"] = _field(f"{base}.{sig}{suffix}")
    table = _episode_table(session_dir, uuid, wanted)
    cols = {name: table.column(name).to_pylist() for name in table.column_names}
    t0 = cols["timestamp"][0]
    out: dict[str, Any] = {
        "uuid": uuid,
        "steps": table.num_rows,
        "time": [t - t0 for t in cols["timestamp"]],
        "success": bool(cols["success"][-1]),
        "truncated": bool(cols.get("truncated", [False])[-1]),
        "instruction": cols.get("instruction", [""])[-1],
        "cameras": cameras,
        "robots": {},
    }
    for robot in robots:
        robot_out: dict[str, Any] = {"step": {}, "control": {}, "robot_error": ""}
        for name, values in cols.items():
            if values and values[0] is not None:
                if name.startswith(f"obs.{robot}."):
                    robot_out["step"][name.split(".", 2)[2]] = values
                elif name.startswith(f"env_action.{robot}."):
                    robot_out["step"]["env_action." + name.split(".", 2)[2]] = values
        errors = [e for e in cols.get(f"info.{robot}.robot_error", []) if e]
        robot_out["robot_error"] = errors[-1] if errors else ""
        base = f"info.{robot}.control_states"
        if f"{base}.timestamp" in cols:
            ts = np.concatenate([np.asarray(v, dtype=np.float64) for v in cols[f"{base}.timestamp"] if v is not None])
            idx = _downsample(len(ts))
            robot_out["control"]["time"] = (ts[idx] - t0).tolist()
            robot_out["control"]["num_samples"] = len(ts)
            for sig in CONTROL_SIGNALS:
                if f"{base}.{sig}" not in cols:
                    continue
                parts = []
                for flat, shape in zip(cols[f"{base}.{sig}"], cols[f"{base}.{sig}_shape"], strict=True):
                    if flat is not None and shape is not None:
                        parts.append(np.asarray(flat, dtype=np.float64).reshape(shape))
                if parts:
                    robot_out["control"][sig] = np.concatenate(parts)[idx].tolist()
        out["robots"][robot] = robot_out
    return out


def frame(session_dir: Path, uuid: str, camera: str, step: int) -> bytes:
    path = f"obs.frames.{camera}.rgb.data"
    dataset = _dataset(session_dir)
    if dataset is None or path not in _struct_paths(dataset.schema):
        msg = f"unknown camera {camera!r}"
        raise DataError(msg)
    table = dataset.to_table(
        columns={"data": _field(path)}, filter=(ds.field("uuid") == uuid) & (ds.field("step") == step)
    )
    if table.num_rows == 0:
        msg = f"no frame for step {step}"
        raise DataError(msg)
    return table.column("data")[0].as_py()


def _files_with(session_dir: Path, uuid: str) -> list[Path]:
    out = []
    for f in _files(session_dir):
        uuids = pq.read_table(f, columns=["uuid"]).column("uuid")
        if pc.any(pc.equal(uuids, uuid)).as_py():
            out.append(f)
    return out


def _write_atomic(path: Path, table: pa.Table):
    tmp = path.with_name(f".{path.name}.tmp")
    pq.write_table(table, tmp)
    os.replace(tmp, path)


def relabel_episode(session_dir: Path, uuid: str, success: bool) -> int:
    """Sets the label of an episode the way the recorder does: ``success`` in the last row, False before.
    Returns the number of rewritten files."""
    files = _files_with(session_dir, uuid)
    if not files:
        msg = f"unknown episode {uuid!r}"
        raise DataError(msg)
    last_step = max(
        pc.max(pc.filter(t.column("step"), pc.equal(t.column("uuid"), uuid))).as_py()
        for t in (pq.read_table(f, columns=["uuid", "step"]) for f in files)
    )
    for f in files:
        table = pq.read_table(f)
        is_ep = pc.equal(table.column("uuid"), uuid)
        is_last = pc.and_(is_ep, pc.equal(table.column("step"), last_step))
        new_success = pc.if_else(is_ep, pc.and_(is_last, pa.scalar(success)), table.column("success"))
        idx = table.schema.get_field_index("success")
        table = table.set_column(idx, table.schema.field(idx), new_success.cast(table.schema.field(idx).type))
        _write_atomic(f, table)
    return len(files)


def delete_episode(session_dir: Path, uuid: str) -> int:
    """Removes all rows of an episode. Returns the number of touched files."""
    files = _files_with(session_dir, uuid)
    if not files:
        msg = f"unknown episode {uuid!r}"
        raise DataError(msg)
    for f in files:
        table = pq.read_table(f)
        keep = table.filter(pc.not_equal(table.column("uuid"), uuid))
        if keep.num_rows == 0:
            f.unlink()
        else:
            _write_atomic(f, keep)
    return len(files)
