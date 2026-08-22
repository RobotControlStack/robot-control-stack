#!/usr/bin/env python3
"""Run a TAM checkpoint on one fixed, hash-verified input and print the output.

Two subcommands:

    make-input   write the deterministic golden input file (numpy + MuJoCo, no JAX)
    run          load the checkpoint, drive it with that input, print the result

To compare a workstation against the robot workstation, build the input once,
copy the file to both machines, run it on each, and diff the two printed
outputs. The input file's arrays are SHA-256 checked on load, so "both machines
used the same input" is proven rather than assumed.

Input conventions, which the golden file follows and your controller must match:

* The history stream carries ``tau_cmd`` as the **gravity-free** commanded
  torque plus a separate ``gravity`` term. ``push_window`` forms the model-space
  torque as ``tau_cmd + gravity`` (the released checkpoints have
  ``ideal_model_has_gravity=True``).
* The adaptor window torque is **model space** (gravity comp already included)
  and mixed-convention: rows ``:-1`` are previously applied torque, row ``-1``
  is the current pre-TAM desired torque. ``SimAdaptorJointwiseFlat`` zeroes that
  last row out of the history stem and routes it through the command head, so an
  off-by-one there changes the answer silently.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if SRC.is_dir() and str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

INPUT_SCHEMA = "tam_golden_input/1"

# ---------------------------------------------------------------------------
# Golden trajectory definition. Closed-form constants only, no RNG anywhere, so
# the same file is produced on any numpy version.
# ---------------------------------------------------------------------------
DEFAULT_DOF = 7
# Franka Panda home pose, inside the joint ranges of the packaged ideal MJCF.
HOME_Q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float64)
SINE_AMP = np.array([0.30, 0.25, 0.30, 0.30, 0.35, 0.30, 0.40], dtype=np.float64)
SINE_FREQ_HZ = np.array([0.31, 0.47, 0.23, 0.53, 0.71, 0.41, 0.61], dtype=np.float64)
SINE_PHASE = np.array([0.0, 0.6, 1.2, 1.8, 2.4, 3.0, 3.6], dtype=np.float64)
# Makes the newest adaptor row distinguishable from a plain continuation of the
# applied-torque history, so an off-by-one in the window cannot pass unnoticed.
TAU_DES_OFFSET = np.array([0.5, -0.4, 0.3, -0.2, 0.1, -0.05, 0.02], dtype=np.float64)
ADAPTOR_WINDOW_ROWS = 32
# Analytic fallback torques when MuJoCo inverse dynamics is not used.
ANALYTIC_GRAVITY_AMP = np.array([1.5, 12.0, 1.0, 9.0, 0.6, 1.4, 0.15], dtype=np.float64)
ANALYTIC_TAU_AMP = np.array([4.0, 6.0, 3.0, 5.0, 1.2, 1.0, 0.4], dtype=np.float64)

# Geometry of the released checkpoints, used only to forecast token counts in
# `make-input`, which does not load a checkpoint. `run` reports the real values.
REF_PATCH_SIZE = 400
REF_PATCH_STRIDE = 200
REF_CONTEXT_HALF = 50
REF_ATTENTION_HISTORY_S = 4.0

STRESS_ZERO_TAU_SPAN = (1200, 1400)   # controller idle: zero commanded torque
STRESS_DROP_SPAN = (2600, 2650)       # dropped samples: hole in the 1 kHz grid


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _sha256_array(arr: np.ndarray) -> str:
    """Hash an array's raw bytes, with dtype and shape folded in."""
    a = np.ascontiguousarray(arr)
    return hashlib.sha256(f"{a.dtype.str}|{a.shape}|".encode() + a.tobytes()).hexdigest()


def _tolist(arr: np.ndarray) -> list:
    """float32 -> python floats that round-trip exactly through JSON."""
    return np.asarray(arr, dtype=np.float32).astype(float).tolist()


def _fmt(vec) -> str:
    """Full float32 precision, for diffing two terminals digit by digit."""
    return " ".join(f"{float(v):+.9e}" for v in np.asarray(vec).reshape(-1))


def _token_forecast(n_samples: int, dt: float) -> dict:
    """How many history tokens a stream of this length produces, and why.

    A token is emitted every ``patch_stride`` samples once the decode patch
    (``patch_size + 2 * context_half``) is full::

        tokens = 1 + (n_samples - decode_patch_size) // patch_stride

    The attention window bounds how many of those tokens the encoder can still
    see. Streams longer than that window exercise the cache-eviction and RoPE
    rebase path that a long run on hardware hits; shorter streams never do.
    """
    decode_patch = REF_PATCH_SIZE + 2 * REF_CONTEXT_HALF
    tokens = 0 if n_samples < decode_patch else 1 + (n_samples - decode_patch) // REF_PATCH_STRIDE
    attn_samples = int(round(REF_ATTENTION_HISTORY_S / dt))
    attn_tokens = (
        1 if attn_samples <= decode_patch
        else 1 + (attn_samples - decode_patch) // REF_PATCH_STRIDE
    )
    return {
        "decode_patch_size": decode_patch,
        "tokens": int(tokens),
        "attention_tokens": int(attn_tokens),
        "evictions": int(max(0, tokens - attn_tokens)),
        "duration_s": n_samples * dt,
    }


# ---------------------------------------------------------------------------
# make-input
# ---------------------------------------------------------------------------
def _reference_trajectory(n_samples: int, dt: float, dof: int):
    """Closed-form q/qd/qdd. qd and qdd are the exact derivatives of q."""
    t = np.arange(n_samples, dtype=np.float64) * float(dt)
    amp = SINE_AMP[:dof]
    w = 2.0 * np.pi * SINE_FREQ_HZ[:dof]
    ph = SINE_PHASE[:dof]
    arg = t[:, None] * w[None, :] + ph[None, :]
    q = HOME_Q[None, :dof] + amp[None, :] * np.sin(arg)
    qd = amp[None, :] * w[None, :] * np.cos(arg)
    qdd = -amp[None, :] * (w[None, :] ** 2) * np.sin(arg)
    return t, q, qd, qdd


def _torques_mujoco(q, qd, qdd, xml_path: Path):
    """gravity = bias torque at zero velocity; tau_total = inverse dynamics."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    # Contacts off: inverse dynamics must be a pure function of (q, qd, qdd).
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    data = mujoco.MjData(model)
    nv = int(model.nv)
    if nv != q.shape[1]:
        raise ValueError(
            f"{xml_path} has nv={nv} but the golden trajectory has {q.shape[1]} DoF."
        )
    gravity = np.zeros_like(q)
    tau_total = np.zeros_like(q)
    for i in range(q.shape[0]):
        data.qpos[:nv] = q[i]
        data.qvel[:nv] = 0.0
        mujoco.mj_forward(model, data)
        gravity[i] = np.asarray(data.qfrc_bias[:nv], dtype=np.float64)

        data.qpos[:nv] = q[i]
        data.qvel[:nv] = qd[i]
        data.qacc[:nv] = qdd[i]
        mujoco.mj_inverse(model, data)
        tau_total[i] = np.asarray(data.qfrc_inverse[:nv], dtype=np.float64)
    meta = {
        "dynamics": "mujoco",
        "mujoco": mujoco.__version__,
        "xml": str(xml_path),
        "xml_sha256": _sha256_file(xml_path),
        "contacts_disabled": True,
    }
    return gravity, tau_total, meta


def _torques_analytic(t, q, qd, qdd, dof: int):
    """MuJoCo-free stand-in with realistic magnitudes and no external deps."""
    w = 2.0 * np.pi * SINE_FREQ_HZ[:dof]
    ph = SINE_PHASE[:dof]
    arg = t[:, None] * w[None, :] + ph[None, :]
    gravity = ANALYTIC_GRAVITY_AMP[None, :dof] * np.cos(arg)
    tau_cmd = ANALYTIC_TAU_AMP[None, :dof] * np.sin(1.7 * arg + 0.4)
    return gravity, tau_cmd + gravity, {"dynamics": "analytic"}


def _default_xml() -> Path:
    try:
        from simadaptor.assets import default_panda_xml

        return Path(default_panda_xml())
    except Exception:
        fallback = ROOT / "assets" / "franka_panda" / "panda_pandagripper.xml"
        if fallback.is_file():
            return fallback
        raise SystemExit(
            "Could not locate the packaged ideal-model MJCF; pass --xml explicitly."
        )


def make_input(args: argparse.Namespace) -> int:
    dof = int(args.dof)
    n = int(args.samples)
    dt = float(args.dt)
    if dof < 1 or dof > DEFAULT_DOF:
        raise SystemExit(f"--dof must be in [1, {DEFAULT_DOF}] (golden constants are 7-DoF).")

    forecast = _token_forecast(n, dt)
    if forecast["tokens"] < 1:
        raise SystemExit(
            f"--samples {n} is shorter than the decode patch "
            f"({forecast['decode_patch_size']} samples), so no history token would ever be "
            "emitted. Use at least that many samples."
        )

    t, q, qd, qdd = _reference_trajectory(n, dt, dof)

    if args.dynamics == "mujoco":
        xml_path = Path(args.xml).expanduser().resolve() if args.xml else _default_xml()
        try:
            gravity, tau_total, dyn_meta = _torques_mujoco(q, qd, qdd, xml_path)
        except ImportError as exc:
            raise SystemExit(
                f"--dynamics mujoco needs the mujoco package ({exc}). "
                "Re-run with --dynamics analytic, or generate the input on a machine "
                "that has MuJoCo installed (the file is portable once written)."
            )
    else:
        gravity, tau_total, dyn_meta = _torques_analytic(t, q, qd, qdd, dof)

    tau_cmd = tau_total - gravity  # gravity-free commanded torque, as the controller logs it

    q32 = q.astype(np.float32)
    qd32 = qd.astype(np.float32)
    tau_cmd32 = tau_cmd.astype(np.float32)
    grav32 = gravity.astype(np.float32)
    keep = np.ones((n,), dtype=np.float32)
    t64 = t.copy()

    profile_meta: dict[str, Any] = {"profile": args.profile}
    if args.profile == "stress":
        lo, hi = STRESS_ZERO_TAU_SPAN
        hi = min(hi, n)
        if lo < hi:
            # Exactly-zero commanded torque marks the rows invalid for history
            # (zero_torque_history_keep_mask), which is the controller-idle path.
            tau_cmd32[lo:hi] = 0.0
            keep[lo:hi] = 0.0
        drop_lo, drop_hi = STRESS_DROP_SPAN
        drop_hi = min(drop_hi, n)
        profile_meta.update(
            zero_tau_span=[int(lo), int(hi)],
            dropped_span=[int(drop_lo), int(drop_hi)],
        )
        if drop_lo < drop_hi:
            mask = np.ones((n,), dtype=bool)
            mask[drop_lo:drop_hi] = False
            t64, q32, qd32, tau_cmd32, grav32, keep = (
                t64[mask], q32[mask], qd32[mask], tau_cmd32[mask], grav32[mask], keep[mask],
            )

    # Adaptor window: the tail of the same stream, in model space, with the
    # newest row offset so it is unmistakably the pre-TAM desired torque.
    rows = min(ADAPTOR_WINDOW_ROWS, q32.shape[0])
    adaptor_q = q32[-rows:].copy()
    adaptor_dq = qd32[-rows:].copy()
    adaptor_tau_model = (tau_cmd32[-rows:] + grav32[-rows:]).astype(np.float32)
    adaptor_tau_model[-1] = (
        adaptor_tau_model[-1].astype(np.float64) + TAU_DES_OFFSET[:dof]
    ).astype(np.float32)

    arrays = {
        "t": t64,
        "q": q32,
        "qd": qd32,
        "tau_cmd": tau_cmd32,
        "gravity": grav32,
        "keep_mask": keep,
        "adaptor_q": adaptor_q,
        "adaptor_dq": adaptor_dq,
        "adaptor_tau_model": adaptor_tau_model,
    }
    meta = {
        "schema": INPUT_SCHEMA,
        "dof": dof,
        "dt": dt,
        "requested_samples": n,
        "stream_rows": int(q32.shape[0]),
        "adaptor_window_rows": int(rows),
        "token_forecast": forecast,
        "tau_des_offset": _tolist(TAU_DES_OFFSET[:dof]),
        "home_q": _tolist(HOME_Q[:dof]),
        "sine_amp": _tolist(SINE_AMP[:dof]),
        "sine_freq_hz": _tolist(SINE_FREQ_HZ[:dof]),
        "sine_phase": _tolist(SINE_PHASE[:dof]),
        "generator": {"script_sha256": _sha256_file(Path(__file__).resolve()), **dyn_meta},
        **profile_meta,
        "array_sha256": {k: _sha256_array(v) for k, v in arrays.items()},
        "notes": {
            "tau_cmd": "gravity-free commanded torque; the runtime adds `gravity` to get model space",
            "adaptor_tau_model": "model-space torque window; row -1 is the pre-TAM desired torque, rows :-1 are previously applied torque",
        },
    }

    out = Path(args.out).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, meta_json=np.array(json.dumps(meta, indent=2)), **arrays)
    meta["file"] = {"path": str(out), "sha256": _sha256_file(out)}
    out.with_suffix(".json").write_text(json.dumps(meta, indent=2) + "\n")

    print(f"[make-input] wrote {out}")
    print(f"[make-input] file sha256 = {meta['file']['sha256']}")
    print(
        f"[make-input] dof={dof} dt={dt} stream_rows={meta['stream_rows']} "
        f"profile={args.profile} dynamics={dyn_meta['dynamics']}"
    )
    print(
        f"[make-input] forecast ({forecast['duration_s']:.3f} s at {1.0 / dt:.0f} Hz, "
        f"released-checkpoint geometry): {forecast['tokens']} history tokens, "
        f"attention window holds {forecast['attention_tokens']}, "
        f"{forecast['evictions']} cache eviction(s)"
    )
    return 0


def _load_input(path: Path) -> tuple[dict, dict]:
    data = np.load(path, allow_pickle=False)
    if "meta_json" not in data.files:
        raise SystemExit(f"{path} is not a golden input file (no meta_json).")
    meta = json.loads(str(data["meta_json"]))
    if meta.get("schema") != INPUT_SCHEMA:
        raise SystemExit(f"{path} has schema {meta.get('schema')!r}, expected {INPUT_SCHEMA!r}.")
    arrays = {k: np.asarray(data[k]) for k in data.files if k != "meta_json"}
    stored = meta.get("array_sha256", {})
    bad = [k for k, v in arrays.items() if stored.get(k) and _sha256_array(v) != stored[k]]
    if bad:
        raise SystemExit(f"{path}: array hash mismatch for {bad}; the file is corrupt.")
    return meta, arrays


# ---------------------------------------------------------------------------
# run
# ---------------------------------------------------------------------------
def _squeeze_embedding(emb: Any) -> np.ndarray:
    """Runtime embedding -> [DoF, C] (jointwise) or [C] (global)."""
    a = np.asarray(emb, dtype=np.float32)
    if a.ndim == 3:
        if a.shape[0] != 1:
            raise ValueError(f"expected batch size 1, got embedding shape {a.shape}")
        return a[0]
    if a.ndim == 2 and a.shape[0] == 1:
        return a[0]
    return a


def _stream_history(runtime, arrays: dict, chunk: int) -> tuple[Optional[np.ndarray], int]:
    """Push the whole stream and return the final latent plus the token count.

    Chunking is a batching detail only: push_window places samples on a dense
    grid by timestamp and decodes fixed-size patches at fixed absolute indices,
    so the latent does not depend on how the stream is sliced.
    """
    t = np.asarray(arrays["t"], dtype=np.float64)
    q = np.asarray(arrays["q"], dtype=np.float32)
    qd = np.asarray(arrays["qd"], dtype=np.float32)
    tau_cmd = np.asarray(arrays["tau_cmd"], dtype=np.float32)
    gravity = np.asarray(arrays["gravity"], dtype=np.float32)
    keep = np.asarray(arrays["keep_mask"], dtype=np.float32)

    runtime.reset()
    latest = None
    tokens = 0
    stride = max(int(getattr(runtime, "_patch_stride", 1) or 1), 1)
    for start in range(0, t.shape[0], chunk):
        end = min(t.shape[0], start + chunk)
        before = getattr(runtime, "_next_emit_idx", None)
        emb = runtime.push_window(
            timestamps=t[start:end],
            q=q[start:end],
            qd=qd[start:end],
            tau=tau_cmd[start:end],
            gravity=gravity[start:end],
            keep_mask=keep[start:end],
        )
        after = getattr(runtime, "_next_emit_idx", None)
        if emb is not None:
            latest = _squeeze_embedding(emb)
            tokens += (
                max(1, int((after - before) // stride))
                if before is not None and after is not None and after > before
                else 1
            )
    return latest, tokens


def run(args: argparse.Namespace) -> int:
    input_path = Path(args.input).expanduser().resolve()
    meta, arrays = _load_input(input_path)

    import jax.numpy as jnp

    from simadaptor.deploy.history_runtime import RealTimeHistoryAdaptor

    # ckpt_path=None is not "unset": it is the instruction from_checkpoint() takes
    # to fetch the packaged default (dagger_applied_8850). The resolved directory
    # is printed below, once the loader has told us what it picked.
    runtime = RealTimeHistoryAdaptor.from_checkpoint(
        args.ckpt_path,
        xml_path=args.xml,
        expected_dt=float(meta["dt"]),
        attention_history_s=float(args.attention_history_s),
        require_history_torque_mode=None if args.allow_any_torque_mode else "applied",
    )
    inf = runtime.inf
    dof = int(getattr(runtime, "_dof"))
    if dof != int(meta["dof"]):
        raise SystemExit(
            f"checkpoint DoF {dof} != golden input DoF {meta['dof']}; regenerate the input "
            f"with --dof {dof}."
        )

    seq_length = int(inf.adaptor_seq_length)
    win_rows = int(arrays["adaptor_q"].shape[0])
    if win_rows < seq_length:
        raise SystemExit(
            f"golden input carries only {win_rows} adaptor rows but the checkpoint needs "
            f"adaptor_seq_length={seq_length}; regenerate the input."
        )
    win_q = np.asarray(arrays["adaptor_q"][-seq_length:], dtype=np.float32)
    win_dq = np.asarray(arrays["adaptor_dq"][-seq_length:], dtype=np.float32)
    win_tau = np.asarray(arrays["adaptor_tau_model"][-seq_length:], dtype=np.float32)

    z_final, tokens = _stream_history(runtime, arrays, int(args.stream_chunk))
    # print(f"Z FINAL: {z_final}")
    if z_final is None:
        raise SystemExit(
            "No history embedding was produced; the golden input is too short for "
            f"decode_patch_size={getattr(runtime, '_decode_patch_size')}."
        )
    # Same stream again: on one machine this must be exactly 0, or nothing about
    # a cross-machine comparison means anything.
    z_repeat, _ = _stream_history(runtime, arrays, int(args.stream_chunk))
    repeat_diff = float(np.abs(np.asarray(z_repeat) - z_final).max())

    def adaptor(latent: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        tau_out = np.asarray(
            inf.adaptor(
                jnp.asarray(win_q),
                jnp.asarray(win_dq),
                jnp.asarray(win_tau),
                jnp.asarray(latent),
            ),
            dtype=np.float32,
        )
        delta = (tau_out.astype(np.float64) - win_tau[-1].astype(np.float64)).astype(np.float32)
        return tau_out, delta

    tau_out, delta_tau = adaptor(z_final)
    # Zero latent: reproducible without the history encoder at all, so a
    # mismatch here isolates the adaptor head from the encoder.
    tau_out_zero, delta_tau_zero = adaptor(np.zeros_like(z_final))

    weight_bytes = runtime.adaptor_weight_bytes()
    print()
    print(f"input       {input_path.name}  sha256 {_sha256_file(input_path)}")
    print(f"checkpoint  {inf.ckpt_path}")
    print(f"weights     {len(weight_bytes)} B  sha256 {hashlib.sha256(weight_bytes).hexdigest()}")
    print(
        f"geometry    dof={dof} adaptor_seq_length={seq_length} "
        f"patch={getattr(runtime, '_patch_size')}/{getattr(runtime, '_patch_stride')} "
        f"decode_patch={getattr(runtime, '_decode_patch_size')} "
        f"attention_tokens={getattr(runtime, '_attention_history_tokens')}"
    )
    print(f"latent      {tuple[Any, ...](z_final.shape)} from {tokens} tokens  "
          f"sha256 {_sha256_array(z_final)}")
    print(f"repeat      max_abs_diff {repeat_diff!r}  (must be 0.0)")
    print()
    print(f"tau_des     {_fmt(win_tau[-1])}")
    print(f"delta_tau   {_fmt(delta_tau)}")
    print(f"tau_out     {_fmt(tau_out)}")
    print()
    print(f"zero latent delta_tau   {_fmt(delta_tau_zero)}")
    print(f"zero latent tau_out     {_fmt(tau_out_zero)}")

    if args.save_latent is not None:
        out = Path(args.save_latent).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        # Provenance travels with the latent so two files cannot be mixed up.
        np.savez(
            out,
            z_final=z_final,
            meta_json=np.array(json.dumps({
                "input": str(input_path),
                "input_sha256": _sha256_file(input_path),
                "ckpt_path": str(inf.ckpt_path),
                "adaptor_weight_sha256": hashlib.sha256(weight_bytes).hexdigest(),
                "num_tokens": int(tokens),
                "z_final_sha256": _sha256_array(z_final),
                "attention_history_s": float(args.attention_history_s),
            }, indent=2)),
        )
        print()
        print(f"saved       {out}  (z_final {tuple(z_final.shape)} float32, meta_json)")
    return 0


# ---------------------------------------------------------------------------
# cli
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="tam_golden_check.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    mk = sub.add_parser("make-input", help="write the deterministic golden input file")
    mk.add_argument("--out", type=Path, default=Path("golden_input.npz"))
    mk.add_argument("--dof", type=int, default=DEFAULT_DOF)
    mk.add_argument("--dt", type=float, default=0.001, help="control period of the stream (s)")
    mk.add_argument(
        "--samples", type=int, default=4600,
        help=(
            "stream length in samples. The default 4600 (4.6 s at 1 kHz) yields 21 history "
            "tokens against an 18-token attention window, so the cache-eviction and RoPE "
            "rebase path a long run on hardware hits is exercised three times. Use 500 for "
            "the smallest possible case: one decode patch, one token, one latent."
        ),
    )
    mk.add_argument("--profile", choices=["basic", "stress"], default="basic",
                    help="stress adds a zero-torque (idle) span and a dropped-sample hole")
    mk.add_argument("--dynamics", choices=["mujoco", "analytic"], default="mujoco",
                    help="mujoco: gravity from qfrc_bias and torque from inverse dynamics")
    mk.add_argument("--xml", type=Path, default=None,
                    help="ideal-model MJCF (default: packaged Panda)")
    mk.set_defaults(func=make_input)

    rn = sub.add_parser("run", help="drive a checkpoint with the golden input and print the output")
    rn.add_argument("--input", type=Path, required=True)
    rn.add_argument("--ckpt-path", type=str, default=None,
                    help="checkpoint_<step> dir; omit to fetch the packaged default, matching "
                         "RealTimeHistoryAdaptor.from_checkpoint()")
    rn.add_argument("--xml", type=str, default=None,
                    help="omit to use the packaged ideal-model MJCF")
    rn.add_argument("--attention-history-s", type=float, default=4.0,
                    help="must match the deployed value (mapping_server default is 4.0)")
    rn.add_argument("--stream-chunk", type=int, default=200,
                    help="samples per push_window call; a batching detail that does not "
                         "change the latent")
    rn.add_argument("--allow-any-torque-mode", action="store_true",
                    help="accept non-applied-torque checkpoints (from_checkpoint enforces applied)")
    rn.add_argument("--save-latent", type=Path, default=None, metavar="PATH",
                    help="write z_final to this .npz (keys: z_final, meta_json); "
                         "omit to write nothing")
    rn.set_defaults(func=run)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
