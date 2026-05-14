from dataclasses import dataclass
from pathlib import Path
from typing import Any

import duckdb
import gymnasium as gym
import numpy as np
from rcs._core.sim import SimConfig
from rcs.envs.base import RelativeTo, SimEnv, SimStateSchema
from rcs.envs.scenes import SimEnvCreator
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.sim.sim import RAW_STATE_ENCODING


def _normalize_sim_state_schema(value: Any) -> SimStateSchema:
    joint_names = [str(item) for item in value["joint_names"]]
    return {
        "joint_names": joint_names,
        "joint_types": [int(item) for item in value["joint_types"]],
        "qpos_sizes": [int(item) for item in value["qpos_sizes"]],
        "qvel_sizes": [int(item) for item in value["qvel_sizes"]],
        "encodings": [str(item) for item in value.get("encodings", [RAW_STATE_ENCODING] * len(joint_names))],
    }


@dataclass(frozen=True)
class RecordedSimStep:
    step: int
    uuid: str
    timestamp: float | None
    observation: dict[str, Any]
    info: dict[str, Any]
    action: Any
    instruction: str
    success: bool

    @property
    def sim_state(self) -> np.ndarray:
        if SimEnv.STATE_KEY in self.info:
            return np.asarray(self.info[SimEnv.STATE_KEY], dtype=np.float64)

        for value in self.info.values():
            if isinstance(value, dict) and SimEnv.STATE_KEY in value:
                return np.asarray(value[SimEnv.STATE_KEY], dtype=np.float64)

        msg = f"Recorded step {self.step} does not contain a sim state in info."
        raise KeyError(msg)

    @property
    def sim_state_schema(self) -> SimStateSchema | None:
        if SimEnv.STATE_SCHEMA_KEY in self.info:
            return _normalize_sim_state_schema(self.info[SimEnv.STATE_SCHEMA_KEY])

        for value in self.info.values():
            if isinstance(value, dict) and SimEnv.STATE_SCHEMA_KEY in value:
                return _normalize_sim_state_schema(value[SimEnv.STATE_SCHEMA_KEY])

        return None


def load_distinct_uuids(dataset_path: Path | str) -> list[str]:
    connection = duckdb.connect()
    try:
        rows = connection.execute(
            "SELECT DISTINCT uuid FROM read_parquet(?) ORDER BY uuid",
            [str(dataset_path)],
        ).fetchall()
    finally:
        connection.close()
    return [str(row[0]) for row in rows]


def load_trajectory(dataset_path: Path | str, trajectory_uuid: str) -> list[RecordedSimStep]:
    connection = duckdb.connect()
    try:
        rows = connection.execute(
            "SELECT uuid, step, timestamp, obs, info, env_action, instruction, success "
            "FROM read_parquet(?) WHERE uuid = ? ORDER BY step",
            [str(dataset_path), trajectory_uuid],
        ).fetchall()
    finally:
        connection.close()

    return [
        RecordedSimStep(
            step=int(row[1]),
            uuid=str(row[0]),
            timestamp=float(row[2]) if row[2] is not None else None,
            observation=row[3],
            info=row[4],
            action=row[5],
            instruction=str(row[6]),
            success=bool(row[7]),
        )
        for row in rows
    ]


def restore_sim_step(env: gym.Env, recorded_step: RecordedSimStep):
    try:
        lead_env = env.get_wrapper_attr("lead_env")
    except AttributeError:
        lead_env = None

    if lead_env is not None:
        lead_env.set_replay_state(recorded_step.sim_state, schema=recorded_step.sim_state_schema)
    else:
        env.get_wrapper_attr("set_replay_state")(recorded_step.sim_state, schema=recorded_step.sim_state_schema)


def replay_trajectory(env: gym.Env, recorded_steps: list[RecordedSimStep], headless: bool):
    if not recorded_steps:
        msg = "No recorded sim states found in the requested trajectory."
        raise ValueError(msg)

    env.reset()
    for recorded_step in recorded_steps:
        restore_sim_step(env, recorded_step)
        env.step(recorded_step.action)
        if not headless:
            env.get_wrapper_attr("sim").sync_gui()
        if recorded_step.success:
            env.get_wrapper_attr("success")()


def replay(
    dataset: Path | str,
    output: Path | str,
    headless: bool = True,
    frequency: int = 30,
    relative_to: str = RelativeTo.CONFIGURED_ORIGIN.name,
    env_id: str = "rcs/duo",
):
    env_creator = gym.envs.registry[env_id].entry_point
    assert isinstance(env_creator, SimEnvCreator), "only rcs envs are supported for replay"

    sim_cfg_data = env_creator.config()
    sim_cfg_data.sim_cfg = SimConfig(
        async_control=True,
        realtime=not headless,
        frequency=frequency,
        max_convergence_steps=500,
    )
    sim_cfg_data.headless = headless
    sim_cfg_data.relative_to = RelativeTo[relative_to.upper()]
    if sim_cfg_data.root_frame_objects is None:
        sim_cfg_data.root_frame_objects = {}

    uuids = load_distinct_uuids(dataset)

    env_rel = env_creator.create_env(sim_cfg_data)
    env_rel = StorageWrapper(
        env=env_rel,
        base_dir=str(output),
        instruction=None,
        batch_size=32,
        max_rows_per_group=100,
        max_rows_per_file=1000,
        always_record=True,
        allow_wrapper_instruction=True,
        success_from_env=False,
    )
    try:
        for uuid in uuids:
            recorded_steps = load_trajectory(dataset, uuid)
            if not recorded_steps:
                continue
            env_rel.get_wrapper_attr("set_instruction")(recorded_steps[0].instruction)
            replay_trajectory(env_rel, recorded_steps, headless)
    finally:
        env_rel.close()
