import copy
import hashlib
import json
import math
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import duckdb
import gymnasium as gym
import numpy as np
from rcs._core.sim import SimConfig
from rcs.envs.base import RelativeTo, SimEnv, SimStateSchema
from rcs.envs.scenes import SimEnvCreator, SimEnvCreatorConfig
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.sim.composer import ModelComposer
from rcs.sim.sim import RAW_STATE_ENCODING
from rcs.sim.visual_randomization import RANDOMIZER_NAMES, VisualRandomizer

ORIGINAL_VARIANT = "original"
VISUAL_RANDOMIZATION_INFO_KEY = "visual_randomization"


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


@dataclass(frozen=True)
class ReplayVariant:
    """A randomized copy of the replayed data.

    Attributes:
        randomizers: names of the visual randomizers that are applied together, see ``RANDOMIZER_NAMES``.
        multiplier: number of copies per source episode, e.g. 1.0 adds one randomized copy of every episode and
            0.5 adds one copy for a deterministic half of the episodes.
    """

    randomizers: frozenset[str]
    multiplier: float = 1.0

    def __post_init__(self):
        if not self.randomizers:
            msg = "A variant needs at least one randomizer, the original data is controlled by keep_original."
            raise ValueError(msg)
        unknown = self.randomizers - set(RANDOMIZER_NAMES)
        if unknown:
            msg = f"Unknown randomizers {sorted(unknown)}, available: {list(RANDOMIZER_NAMES)}"
            raise ValueError(msg)
        if self.multiplier < 0:
            msg = f"Variant multiplier must be non-negative, got {self.multiplier}."
            raise ValueError(msg)

    @property
    def name(self) -> str:
        return "+".join(sorted(self.randomizers))

    @classmethod
    def parse(cls, value: str) -> "ReplayVariant":
        """Parses '<randomizer>[+<randomizer>...][:<multiplier>]', e.g. 'background+material:0.5'."""
        names, _, multiplier = value.partition(":")
        return cls(
            randomizers=frozenset(name.strip() for name in names.split("+") if name.strip()),
            multiplier=float(multiplier) if multiplier else 1.0,
        )


def _stable_int(*parts: Any) -> int:
    digest = hashlib.sha256(":".join(str(part) for part in parts).encode()).digest()
    # 63 bits so that the value fits into a signed int64 parquet column
    return int.from_bytes(digest[:8], "little") & ((1 << 63) - 1)


def variant_seed(seed: int, uuid: str, variant: ReplayVariant, copy_idx: int) -> int:
    """Seed of one randomized copy, stable across runs and independent of the dataset order."""
    return _stable_int(seed, uuid, variant.name, copy_idx)


def variant_copies(uuids: list[str], variant: ReplayVariant, seed: int) -> dict[str, int]:
    """Number of randomized copies per episode, the fractional part selects a deterministic subset of episodes."""
    whole = math.floor(variant.multiplier)
    num_extra = round((variant.multiplier - whole) * len(uuids))
    extra = set(sorted(uuids, key=lambda uuid: _stable_int(seed, variant.name, uuid))[:num_extra])
    return {uuid: whole + (uuid in extra) for uuid in uuids}


class VisualRandomizationInfoWrapper(gym.Wrapper):
    """Adds the variant, seed and sampled visual randomization parameters to the info dict."""

    def __init__(self, env: gym.Env, variant: str, seed: int, params: dict[str, Any]):
        super().__init__(env)
        self._info = {"variant": variant, "seed": seed, "params": json.dumps(params, sort_keys=True)}

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        info[VISUAL_RANDOMIZATION_INFO_KEY] = dict(self._info)
        return obs, reward, terminated, truncated, info

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        obs, info = self.env.reset(seed=seed, options=options)
        info[VISUAL_RANDOMIZATION_INFO_KEY] = dict(self._info)
        return obs, info


def _storage_env(env: gym.Env, output: Path | str) -> StorageWrapper:
    return StorageWrapper(
        env=env,
        base_dir=str(output),
        instruction=None,
        batch_size=32,
        max_rows_per_group=100,
        max_rows_per_file=1000,
        always_record=True,
        allow_wrapper_instruction=True,
        success_from_env=False,
    )


def _replay_episode(env: gym.Env, recorded_steps: list[RecordedSimStep], headless: bool):
    env.get_wrapper_attr("set_instruction")(recorded_steps[0].instruction)
    replay_trajectory(env, recorded_steps, headless)


def _validate_variants(variants: list[ReplayVariant], cfg: SimEnvCreatorConfig):
    names = [variant.name for variant in variants]
    if len(names) != len(set(names)):
        msg = f"Variants must be unique, got {names}. Use the multiplier for more copies of the same variant."
        raise ValueError(msg)
    requested = set().union(*(variant.randomizers for variant in variants))
    configured = cfg.visual_randomization.configured_randomizers() if cfg.visual_randomization is not None else set()
    missing = requested - configured
    if missing:
        msg = (
            f"Variants request the randomizers {sorted(missing)}, but the environment config does not provide "
            "options for them in visual_randomization."
        )
        raise ValueError(msg)


def replay(
    dataset: Path | str,
    output: Path | str,
    headless: bool = True,
    frequency: int = 30,
    relative_to: str = RelativeTo.CONFIGURED_ORIGIN.name,
    env_id: str = "rcs/duo",
    variants: Iterable[ReplayVariant] = (),
    keep_original: bool = True,
    seed: int = 0,
):
    """Replays all episodes of a dataset and records them again, optionally with visually randomized copies.

    Without variants, every episode is replayed once with the environment as configured. With variants, the
    original episodes (if keep_original) are replayed without visual randomization, and each variant adds
    randomized copies, e.g. variants background:1 and background+material:1 result in three times the data.
    The randomizer options are taken from the visual_randomization field of the environment config. Every
    recorded step then contains the variant, seed and sampled parameters under info["visual_randomization"].
    """
    variants = list(variants)
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

    if not variants and not keep_original:
        msg = "Nothing to replay: keep_original is disabled and no variants are given."
        raise ValueError(msg)
    if variants:
        _validate_variants(variants, sim_cfg_data)

    uuids = load_distinct_uuids(dataset)

    if not variants:
        env = _storage_env(env_creator.create_env(sim_cfg_data), output)
        try:
            for uuid in uuids:
                recorded_steps = load_trajectory(dataset, uuid)
                if recorded_steps:
                    _replay_episode(env, recorded_steps, headless)
        finally:
            env.close()
        return

    assert sim_cfg_data.visual_randomization is not None
    randomizer = VisualRandomizer(sim_cfg_data.visual_randomization)
    # randomization is applied explicitly per variant, the original data stays unrandomized
    base_cfg = copy.copy(sim_cfg_data)
    base_cfg.visual_randomization = None
    copies = {variant: variant_copies(uuids, variant, seed) for variant in variants}

    original_env = (
        _storage_env(VisualRandomizationInfoWrapper(env_creator.create_env(base_cfg), ORIGINAL_VARIANT, 0, {}), output)
        if keep_original
        else None
    )
    try:
        for uuid in uuids:
            recorded_steps = load_trajectory(dataset, uuid)
            if not recorded_steps:
                continue
            if original_env is not None:
                _replay_episode(original_env, recorded_steps, headless)
            for variant in variants:
                for copy_idx in range(copies[variant][uuid]):
                    copy_seed = variant_seed(seed, uuid, variant, copy_idx)
                    composer = env_creator.create_model(base_cfg)
                    if not isinstance(composer, ModelComposer):
                        msg = "Visual randomization requires an environment that composes its model with ModelComposer."
                        raise TypeError(msg)
                    params = randomizer.apply(composer.spec, np.random.default_rng(copy_seed), set(variant.randomizers))
                    randomized_env = _storage_env(
                        VisualRandomizationInfoWrapper(
                            env_creator.create_env_from_model(base_cfg, composer), variant.name, copy_seed, params
                        ),
                        output,
                    )
                    try:
                        _replay_episode(randomized_env, recorded_steps, headless)
                    finally:
                        randomized_env.close()
    finally:
        if original_env is not None:
            original_env.close()
