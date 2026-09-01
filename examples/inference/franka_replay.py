"""Replay a recorded LeRobot dataset episode on the real robot.

Purpose: isolate policy vs. execution. We run the *exact* recorded joint
targets (no policy in the loop) and compare what the robot measures against what
the dataset recorded. If the replay reproduces the dataset motion (and grasps
the cube), the sim2real execution is faithful and any task failure lives in the
policy; if the replay itself is off (e.g. arm too high), the gap is in
execution.

It reuses ``franka_tam``'s env/gain/TAM setup verbatim (via ``get_env`` and the
``ModelInference`` TAM helpers) so the execution path matches the policy runs,
and it writes:
  - the 1 kHz C++ controller/TAM debug log (tam_controller_1khz.csv), and
  - a per-frame replay log (replay_frames.csv) pairing each sent target with the
    measured joints and the dataset's own recorded observation.

Dataset: ``~/dataset/compliant_test_flip_pnp_softer/blocks_success`` (soft PD,
pd_mode=0), 25 fps, absolute targets in ``action.joint_target`` (radians).

Usage:
    python franka_replay.py                 # episode 0, TAM as configured
    EPISODE=3 python franka_replay.py       # pick an episode via env var
"""

import json
import logging
import os
import threading
import time
import types
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

# Reuse the exact env/gain/TAM setup and TAM helpers from the policy script so
# the execution path is identical (no drift between replay and live inference).
from franka_tam import InferenceConfig, ModelInference, get_env

logger = logging.getLogger(__name__)
logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s", level=logging.INFO
)

# Override with REPLAY_DATASET=/path/to/<dataset>/blocks_success
DATASET_ROOT = Path(
    os.environ.get(
        "REPLAY_DATASET",
        # str(Path.home() / "dataset/aug13_sim2real_kp40_pnp/blocks_success"),
        # "/home/yejin/code/robot-control-stack/blocksuite/datasets/aug17_red_rotate_cw_no_dist",
        # "/home/tobi/dataset/naman_mp_flip_aug_26/blocks_success",
        "/home/tobi/dataset/fingertip_hover/fingertip_hover_success",
    )
)
ROBOT_KEY = "right"


@dataclass
class ReplayConfig:
    dataset_root: Path = DATASET_ROOT
    episode: int = int(os.environ.get("EPISODE", "0"))
    # Apply franka::limitRate to the final commanded torque (FCI torque-rate
    # reflex). True in every previous replay run (it was inherited from
    # InferenceConfig.rate_limit, which defaults to True, and never overridden).
    rate_limit: bool = True
    # Run the replay with TAM enabled? Start with False to see the raw sim2real
    # execution gap of the bare soft controller, then re-run with True to see
    # whether TAM closes it. (Run both for the aug13 dataset, same episode.)
    tam: bool = True
    # Some datasets store gripper as 0=open/1=closed; the live obs convention in
    # franka_tam is 0=closed/1=open. Flip here if the gripper acts inverted.
    invert_gripper: bool = True
    # Safety: skip a frame's target if any joint would jump more than this in one
    # step (rad). None disables the check (the env still clamps to
    # max_relative_movement). Purely a warning aid.
    warn_step_jump_rad: float | None = np.deg2rad(6)
    # Override the env's per-step joint clamp (deg). The dataset targets jump up
    # to ~21 deg/step (the compliant controller lagged far behind), so the
    # default 5 deg clamp WILL activate. None keeps franka_tam's default; raise
    # it (e.g. 25) to replay the recorded targets more faithfully if the 5 deg
    # clamp is found to desync the motion.
    max_rel_mov_deg: float | None = None


# --- Dataset loading (mirrors the reference notebook) ---------------------------

def _load_episodes_meta(root: Path) -> pd.DataFrame:
    cols = [
        "episode_index",
        "length",
        "tasks",
        "data/chunk_index",
        "data/file_index",
    ]
    frames = [
        pd.read_parquet(f, columns=cols)
        for f in sorted((root / "meta/episodes/chunk-000").glob("*.parquet"))
    ]
    return pd.concat(frames, ignore_index=True).set_index("episode_index")


def _load_episode(root: Path, meta: pd.DataFrame, episode_index: int) -> pd.DataFrame:
    row = meta.loc[episode_index]
    chunk = int(row["data/chunk_index"])
    file = int(row["data/file_index"])
    df = pd.read_parquet(root / f"data/chunk-{chunk:03d}/file-{file:03d}.parquet")
    return df[df["episode_index"] == episode_index].reset_index(drop=True)


def _stack(df: pd.DataFrame, col: str) -> np.ndarray:
    return np.stack(df[col].to_numpy())


def load_trajectory(root: Path, episode: int):
    """Return (q_target[T,7], gripper[T], pd_mode[T], q_obs[T,7], t[T], task).

    Frames with ``action.is_valid == False`` are dropped — these are terminal /
    "done" frames whose ``action.joint_target`` is garbage (e.g. a 130 deg jump)
    and must never be sent to the robot.
    """
    meta = _load_episodes_meta(root)
    ep = _load_episode(root, meta, episode)
    if "action.is_valid" in ep:
        valid = _stack(ep, "action.is_valid").astype(bool).reshape(-1)
        if not valid.all():
            logger.warning("dropping %d invalid frame(s) of %d", (~valid).sum(), len(ep))
            ep = ep[valid].reset_index(drop=True)
    q_target = _stack(ep, "action.joint_target").astype(np.float64)  # absolute rad
    q_obs = _stack(ep, "observation.state.joint_position").astype(np.float64)
    gripper = _stack(ep, "action.gripper_position").astype(np.float64).reshape(-1)
    action = _stack(ep, "action").astype(np.float64)
    pd_mode = action[:, 8] if action.shape[1] > 8 else np.zeros(len(ep))
    t = _stack(ep, "timestamp").astype(np.float64).reshape(-1)
    task = meta.loc[episode, "tasks"]
    return q_target, gripper, pd_mode, q_obs, t, task


# --- Replay ---------------------------------------------------------------------

def main() -> None:
    rcfg = ReplayConfig()
    cfg = InferenceConfig()  # inherits gains, TAM flags, warmup, logging, fps
    cfg.tam = rcfg.tam  # replay decides TAM on/off independently of the policy default
    cfg.rate_limit = rcfg.rate_limit

    # We bypass ModelInference.__init__ (to avoid the policy server), so create
    # the log directory here the same way it does.
    if cfg.tam_debug_log and cfg.tam_log_dir is None:
        cfg.tam_log_dir = os.path.join(os.getcwd(), "tam_logs", time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(cfg.tam_log_dir, exist_ok=True)
        logger.info("replay logs -> %s", cfg.tam_log_dir)

    q_target, gripper, pd_mode, q_obs_ds, t_ds, task = load_trajectory(
        rcfg.dataset_root, rcfg.episode
    )
    n = len(q_target)
    logger.info(
        "episode %d: %d frames (%.1f s @ %d fps), task=%s",
        rcfg.episode, n, n / cfg.fps, cfg.fps, task,
    )
    if not np.allclose(pd_mode, 0.0):
        logger.warning(
            "dataset pd_mode is not all-0 (soft); this dataset should be soft. "
            "min=%.2f max=%.2f", pd_mode.min(), pd_mode.max()
        )
    # Report how often the env's per-step clamp will bite (targets vs clamp).
    clamp_deg = rcfg.max_rel_mov_deg if rcfg.max_rel_mov_deg is not None else np.rad2deg(cfg.max_rel_mov_joints)
    step_jump_deg = np.rad2deg(np.abs(np.diff(q_target, axis=0)).max(axis=1))
    n_clamped = int(np.sum(step_jump_deg > clamp_deg))
    logger.info(
        "per-step target jump: max %.1f deg, p99 %.1f deg; clamp %.1f deg would "
        "engage on %d/%d frames", step_jump_deg.max(), np.percentile(step_jump_deg, 99),
        clamp_deg, n_clamped, len(q_target),
    )
    if rcfg.max_rel_mov_deg is not None:
        cfg.max_rel_mov_joints = float(np.deg2rad(rcfg.max_rel_mov_deg))

    env = get_env(cfg)
    robot = env.get_wrapper_attr("robot")[ROBOT_KEY]

    # Stand up the TAM runtime the same way franka_tam does, without pulling in
    # the policy server. We reuse ModelInference's TAM helpers, so we need a real
    # ModelInference instance (its methods call each other via self, e.g.
    # _init_tam -> _check_ideal_model_alignment). Create one WITHOUT running
    # __init__ (which would open a RemoteAgent to the policy server) and set only
    # the attributes those helpers touch.
    helper = ModelInference.__new__(ModelInference)
    helper._cfg = cfg
    helper.env = env
    helper.tam_runtime = None
    if cfg.tam:
        helper._init_tam()
        helper.warmup_history_encoder()

    replay_log = None
    if cfg.tam_log_dir:
        replay_log = open(os.path.join(cfg.tam_log_dir, "replay_frames.csv"), "w")
        header = ["frame", "t_ds", "gripper_sent"]
        header += [f"q_sent{j}" for j in range(7)]
        header += [f"q_meas{j}" for j in range(7)]
        header += [f"q_ds{j}" for j in range(7)]  # dataset-recorded measured q
        replay_log.write(",".join(header) + "\n")
        replay_log.flush()
        # Save the episode/dataset provenance next to the logs.
        with open(os.path.join(cfg.tam_log_dir, "replay_meta.json"), "w") as f:
            json.dump(
                {
                    "dataset_root": str(rcfg.dataset_root),
                    "episode": rcfg.episode,
                    "n_frames": int(n),
                    "fps": cfg.fps,
                    "tam": bool(cfg.tam),
                    "rate_limit": bool(cfg.rate_limit),
                    "tam_residual_clip": list(cfg.tam_residual_clip)
                    if cfg.tam_residual_clip is not None else None,
                    "task": list(task) if hasattr(task, "__len__") else str(task),
                },
                f, indent=2,
            )

    input("Press Enter to start replay (Ctrl+C to abort)...")
    if cfg.tam:
        threading.Thread(
            target=helper.run_history_encoder,
            name="history_encoder", daemon=True,
        ).start()

    from rcs.utils import SimpleFrameRate

    frame_rate = SimpleFrameRate(cfg.fps)
    with env:
        obs, _ = env.reset()
        if cfg.tam_log_dir:
            robot.set_tam_logging(True)
            logger.info("TAM 1 kHz debug logging enabled")
        try:
            for i in range(n):
                g = 1.0 - gripper[i] if rcfg.invert_gripper else gripper[i]
                act = {
                    ROBOT_KEY: {
                        "joints": q_target[i].astype(np.float32),
                        "gripper": np.array([g], dtype=np.float32),
                    }
                }
                obs, _, _, _, _ = env.step(act)
                if replay_log is not None:
                    q_meas = np.asarray(obs[ROBOT_KEY]["joints"], dtype=np.float64)
                    vals = [i, t_ds[i], g]
                    vals += q_target[i].tolist() + q_meas.tolist() + q_obs_ds[i].tolist()
                    replay_log.write(",".join(f"{v:.6f}" for v in vals) + "\n")
                frame_rate()
            logger.info("replay finished (%d frames)", n)
        except KeyboardInterrupt:
            logger.info("replay aborted by user")
        finally:
            if replay_log is not None:
                replay_log.close()
            if cfg.tam_log_dir:
                try:
                    robot.set_tam_logging(False)
                    robot.stop_control_thread()
                    tag = "tam" if cfg.tam else "notam"
                    path = os.path.join(cfg.tam_log_dir, f"replay_{tag}_controller_1khz.csv")
                    rows = robot.dump_tam_debug_log(path)
                    logger.info("controller log (%s): %d rows -> %s", tag, rows, path)
                except Exception:
                    logger.exception("failed to dump controller log")
            env.reset()
            logger.info("replay logs in %s", cfg.tam_log_dir)


if __name__ == "__main__":
    main()
