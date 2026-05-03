from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Any

import gymnasium as gym
import numpy as np
import pyarrow.compute as pc
import pyarrow.dataset as ds
import typer
from PIL import Image
from rcs.envs.base import ControlMode
from rcs.envs.sim import SimStateObservationWrapper

import rcs  # noqa: F401

app = typer.Typer(help="Replay recorded MuJoCo trajectories from a parquet dataset.")

DATASET_ARGUMENT = typer.Argument(..., exists=True, file_okay=False, dir_okay=True)
ENV_ID_OPTION = typer.Option("rcs/FR3SimplePickUpSim-v0", help="Gymnasium env id used for replay.")
TRAJECTORY_UUID_OPTION = typer.Option(None, help="UUID of the recorded trajectory to replay.")
CAMERA_OPTION = typer.Option([], "--camera", help="Camera names to enable on the replay env.")
RESOLUTION_OPTION = typer.Option((256, 256), help="Replay camera resolution as WIDTH HEIGHT.")
FRAME_RATE_OPTION = typer.Option(0, help="Replay camera frame rate.")
RENDER_MODE_OPTION = typer.Option("human", help="Gym render mode for the replay env.")
CONTROL_MODE_OPTION = typer.Option(ControlMode.CARTESIAN_TRPY.name, help="Control mode name for env creation.")
SLEEP_OPTION = typer.Option(0.0, help="Optional delay between restored states.")
OUTPUT_DIR_OPTION = typer.Option(None, help="Optional directory for re-rendered RGB frames.")
PREFER_DUCKDB_OPTION = typer.Option(True, help="Use duckdb for parquet loading when it is available.")


@dataclass(frozen=True)
class RecordedSimStep:
    step: int
    uuid: str
    timestamp: float | None
    observation: dict[str, Any]

    @property
    def sim_state(self) -> np.ndarray:
        return np.asarray(self.observation[SimStateObservationWrapper.STATE_KEY], dtype=np.float64)

    @property
    def sim_state_spec(self) -> dict[str, Any] | None:
        schema = self.observation.get(SimStateObservationWrapper.STATE_SPEC_KEY)
        return dict(schema) if schema is not None else None


class DuckDBUnavailableError(RuntimeError):
    pass


def _get_duckdb_module():
    try:
        import duckdb
    except ModuleNotFoundError as exc:
        msg = (
            "duckdb is required for the preferred parquet read path but is not installed. "
            "Install the 'duckdb' Python package or rely on the pyarrow fallback in library calls."
        )
        raise DuckDBUnavailableError(msg) from exc
    return duckdb


def _load_distinct_uuids_with_duckdb(dataset_path: Path) -> list[str]:
    duckdb = _get_duckdb_module()
    connection = duckdb.connect()
    try:
        rows = connection.execute(
            "SELECT DISTINCT uuid FROM read_parquet(?) ORDER BY uuid",
            [str(dataset_path)],
        ).fetchall()
    finally:
        connection.close()
    return [row[0] for row in rows]


def _load_distinct_uuids_with_pyarrow(dataset_path: Path) -> list[str]:
    dataset = ds.dataset(str(dataset_path), format="parquet")
    uuids = dataset.to_table(columns=["uuid"])["uuid"]
    return sorted(str(uuid) for uuid in pc.unique(uuids).to_pylist())


def list_trajectory_ids(dataset_path: Path, prefer_duckdb: bool = True) -> list[str]:
    if prefer_duckdb:
        try:
            return _load_distinct_uuids_with_duckdb(dataset_path)
        except DuckDBUnavailableError:
            pass
    return _load_distinct_uuids_with_pyarrow(dataset_path)


def _load_trajectory_with_duckdb(dataset_path: Path, trajectory_uuid: str) -> list[RecordedSimStep]:
    duckdb = _get_duckdb_module()
    connection = duckdb.connect()
    try:
        table = connection.execute(
            "SELECT uuid, step, timestamp, obs FROM read_parquet(?) WHERE uuid = ? ORDER BY step",
            [str(dataset_path), trajectory_uuid],
        ).to_arrow_table()
    finally:
        connection.close()
    return [
        RecordedSimStep(
            step=int(row["step"]),
            uuid=str(row["uuid"]),
            timestamp=float(row["timestamp"]) if row["timestamp"] is not None else None,
            observation=row["obs"],
        )
        for row in table.to_pylist()
    ]


def _load_trajectory_with_pyarrow(dataset_path: Path, trajectory_uuid: str) -> list[RecordedSimStep]:
    dataset = ds.dataset(str(dataset_path), format="parquet")
    table = dataset.to_table(filter=pc.field("uuid") == trajectory_uuid, columns=["uuid", "step", "timestamp", "obs"])
    rows = table.sort_by([("step", "ascending")]).to_pylist()
    return [
        RecordedSimStep(
            step=int(row["step"]),
            uuid=str(row["uuid"]),
            timestamp=float(row["timestamp"]) if row["timestamp"] is not None else None,
            observation=row["obs"],
        )
        for row in rows
    ]


def load_trajectory(dataset_path: Path, trajectory_uuid: str, prefer_duckdb: bool = True) -> list[RecordedSimStep]:
    if prefer_duckdb:
        try:
            return _load_trajectory_with_duckdb(dataset_path, trajectory_uuid)
        except DuckDBUnavailableError:
            pass
    return _load_trajectory_with_pyarrow(dataset_path, trajectory_uuid)


def resolve_trajectory_uuid(dataset_path: Path, trajectory_uuid: str | None, prefer_duckdb: bool = True) -> str:
    if trajectory_uuid is not None:
        return trajectory_uuid
    available_uuids = list_trajectory_ids(dataset_path, prefer_duckdb=prefer_duckdb)
    if len(available_uuids) == 1:
        return available_uuids[0]
    msg = (
        f"Dataset {dataset_path} contains {len(available_uuids)} trajectories. "
        f"Pass --trajectory-uuid and choose one of: {available_uuids}"
    )
    raise ValueError(msg)


def restore_sim_step(
    env: gym.Env,
    recorded_step: RecordedSimStep,
    sim_state_spec: dict[str, Any] | None = None,
):
    resolved_spec = sim_state_spec or recorded_step.sim_state_spec
    if resolved_spec is None:
        msg = "Recorded sim state is missing its schema."
        raise ValueError(msg)

    sim = env.get_wrapper_attr("sim")
    sim.set_state(recorded_step.sim_state, spec=resolved_spec)


def collect_rgb_frames(env: gym.Env) -> dict[str, np.ndarray]:
    try:
        camera_set = env.get_wrapper_attr("camera_set")
    except AttributeError:
        return {}

    frameset = camera_set.get_latest_frames()
    if frameset is None:
        return {}

    rgb_frames: dict[str, np.ndarray] = {}
    for camera_name, frame in frameset.frames.items():
        lower_name = camera_name.lower()
        if "digit" in lower_name or "tactile" in lower_name:
            continue
        rgb_frames[camera_name] = np.asarray(frame.camera.color.data)
    return rgb_frames


def save_rgb_frames(output_dir: Path, recorded_step: RecordedSimStep, rgb_frames: dict[str, np.ndarray]):
    output_dir.mkdir(parents=True, exist_ok=True)
    for camera_name, rgb_frame in rgb_frames.items():
        Image.fromarray(rgb_frame).save(output_dir / f"step-{recorded_step.step:06d}-{camera_name}.png")


def replay_trajectory(
    env: gym.Env,
    recorded_steps: list[RecordedSimStep],
    *,
    sleep_s: float = 0.0,
    output_dir: Path | None = None,
):
    if not recorded_steps:
        msg = "No recorded sim states found in the requested trajectory."
        raise ValueError(msg)

    sim_state_spec = next(
        (recorded_step.sim_state_spec for recorded_step in recorded_steps if recorded_step.sim_state_spec), None
    )

    env.reset()
    for recorded_step in recorded_steps:
        restore_sim_step(env, recorded_step, sim_state_spec=sim_state_spec)
        if output_dir is not None:
            save_rgb_frames(output_dir, recorded_step, collect_rgb_frames(env))
        if sleep_s > 0:
            time.sleep(sleep_s)


@app.command()
def replay(
    dataset: Annotated[Path, DATASET_ARGUMENT],
    env_id: Annotated[str, ENV_ID_OPTION],
    trajectory_uuid: Annotated[str | None, TRAJECTORY_UUID_OPTION],
    camera: Annotated[list[str], CAMERA_OPTION],
    resolution: Annotated[tuple[int, int], RESOLUTION_OPTION],
    frame_rate: Annotated[int, FRAME_RATE_OPTION],
    render_mode: Annotated[str, RENDER_MODE_OPTION],
    control_mode: Annotated[str, CONTROL_MODE_OPTION],
    sleep_s: Annotated[float, SLEEP_OPTION],
    output_dir: Annotated[Path | None, OUTPUT_DIR_OPTION],
    prefer_duckdb: Annotated[bool, PREFER_DUCKDB_OPTION],
):
    resolved_uuid = resolve_trajectory_uuid(dataset, trajectory_uuid, prefer_duckdb=prefer_duckdb)
    env = gym.make(
        env_id,
        render_mode=render_mode,
        control_mode=ControlMode[control_mode],
        resolution=resolution,
        frame_rate=frame_rate,
        cam_list=camera,
    )
    try:
        recorded_steps = load_trajectory(dataset, resolved_uuid, prefer_duckdb=prefer_duckdb)
        replay_trajectory(env, recorded_steps, sleep_s=sleep_s, output_dir=output_dir)
    finally:
        env.close()

    typer.echo(f"Replayed {len(recorded_steps)} steps from trajectory {resolved_uuid}.")
    if output_dir is not None:
        typer.echo(f"Saved re-rendered RGB frames to {output_dir}.")


if __name__ == "__main__":
    app()
