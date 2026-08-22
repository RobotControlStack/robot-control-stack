"""Plot the TAM debug logs written by franka_tam.py.

Usage:
    # single run (full diagnostic):
    python plot_tam_log.py tam_logs/<timestamp>

    # compare a TAM run against a no-TAM baseline (overlay):
    python plot_tam_log.py tam_logs/<tam_run> tam_logs/<notam_run>

Each directory is expected to hold one <tag>_controller_1khz.csv (tag is "tam"
or "notam") and, for TAM runs, tam_history_encoder.csv. Figures are saved next
to the first CSV.
"""

import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


def _cols(data, prefix):
    """Return the 7 per-joint columns named prefix0..prefix6 as an [N, 7] array."""
    return np.stack([data[f"{prefix}{j}"] for j in range(7)], axis=1)


def _find_ctrl_csv(path: str) -> str:
    """Accept either a directory or a CSV path; return the controller CSV path."""
    if os.path.isfile(path):
        return path
    hits = sorted(glob.glob(os.path.join(path, "*_controller_1khz.csv")))
    if not hits:
        raise FileNotFoundError(f"no *_controller_1khz.csv under {path}")
    return hits[0]


def _load(path: str):
    csv = _find_ctrl_csv(path)
    d = np.genfromtxt(csv, delimiter=",", names=True)
    tag = "tam" if os.path.basename(csv).startswith("tam") else "notam"
    return d, tag, csv


def _rms_tracking(d) -> np.ndarray:
    err = _cols(d, "desired_q") - _cols(d, "q")
    return np.rad2deg(np.sqrt(np.mean(err**2, axis=0)))


def diagnostic(path: str) -> None:
    d, tag, csv = _load(path)
    t = d["t"] - d["t"][0]

    tau_pd = _cols(d, "tau_pd")
    delta_raw = _cols(d, "delta_raw")
    delta_clip = _cols(d, "delta_clipped")
    delta_app = _cols(d, "delta_applied")
    tau_final = _cols(d, "tau_final")
    track_err = _cols(d, "desired_q") - _cols(d, "q")

    # Per-tick change of the applied residual: the quantity the torque-rate
    # limiter fights. Spikes here are the discontinuity hypothesis.
    ddelta = np.vstack([np.zeros((1, 7)), np.diff(delta_app, axis=0)])

    fig, ax = plt.subplots(4, 2, figsize=(16, 14), sharex=True)

    for j in range(7):
        ax[0, 0].plot(t, delta_app[:, j], label=f"j{j}")
    ax[0, 0].set_title("TAM residual applied (post-clip, post-ramp) [Nm]")
    ax[0, 0].legend(ncol=7, fontsize=7)

    for j in range(7):
        ax[0, 1].plot(t, delta_raw[:, j], label=f"j{j}")
    ax[0, 1].set_title("TAM residual RAW (pre-clip) [Nm] — clip vs saturation")

    ax[1, 0].plot(t, np.abs(ddelta).max(axis=1), color="tab:red")
    ax[1, 0].set_title("max |Δ residual per tick| [Nm/ms] (discontinuity)")
    ax[1, 0].axhline(1.0, color="k", ls="--", lw=0.8, label="1 Nm/ms")
    ax[1, 0].legend(fontsize=8)

    for j in range(7):
        ax[1, 1].plot(t, np.rad2deg(track_err[:, j]), label=f"j{j}")
    ax[1, 1].set_title("tracking error desired_q - q [deg]")

    for j in range(7):
        ax[2, 0].plot(t, tau_pd[:, j], label=f"j{j}")
    ax[2, 0].set_title("controller torque BEFORE TAM (tau_pd) [Nm]")

    for j in range(7):
        ax[2, 1].plot(t, tau_final[:, j], label=f"j{j}")
    ax[2, 1].set_title("final commanded torque (tau_final) [Nm]")

    ax[3, 0].plot(t, d["max_ood_tau"], color="tab:purple")
    ax[3, 0].set_title("max |z-score| of now-row torque vs norm_stats (OOD)")
    ax[3, 0].axhline(3.0, color="k", ls="--", lw=0.8, label="3σ")
    ax[3, 0].legend(fontsize=8)
    ax[3, 0].set_xlabel("t [s]")

    ax2 = ax[3, 1]
    ax2.plot(t, d["latent_age"] * 1e3, color="tab:blue", label="latent age [ms]")
    ax2.plot(t, d["ramp"] * 100.0, color="tab:green", label="ramp [%]")
    ax2.plot(t, d["compute_us"] / 10.0, color="tab:orange", label="compute [/10 us]")
    ax2.set_title("latent staleness / ramp / loop compute")
    ax2.legend(fontsize=8)
    ax2.set_xlabel("t [s]")

    print(f"[{tag}] {csv}")
    print(f"  rows={len(t)}  duration={t[-1]:.1f}s")
    print(f"  model_present={float(np.mean(d['model_present'])):.2f}  "
          f"window_ok={float(np.mean(d['window_ok'])):.2f}  "
          f"active={float(np.mean(d['active'])):.2f}")
    print(f"  history_steps={int(np.median(d['history_steps']))}  "
          f"window_dt(ms) median={np.median(d['window_dt']) * 1e3:.3f}")
    print(f"  max applied residual/joint [Nm] = {np.round(np.abs(delta_app).max(axis=0), 2)}")
    print(f"  clip saturation frac/joint = "
          f"{np.round(np.mean(np.abs(delta_raw) > np.abs(delta_clip) + 1e-6, axis=0), 3)}")
    print(f"  max |Δresidual/tick| [Nm/ms] = {np.abs(ddelta).max():.3f}")
    print(f"  latent_age(ms): median={np.median(d['latent_age']) * 1e3:.1f}  "
          f"max={np.max(d['latent_age']) * 1e3:.1f}")
    print(f"  max_ood_tau: median={np.median(d['max_ood_tau']):.2f}  max={np.max(d['max_ood_tau']):.2f}")
    print(f"  compute_us: median={np.median(d['compute_us']):.1f}  "
          f"p99={np.percentile(d['compute_us'], 99):.1f}  max={np.max(d['compute_us']):.1f}")
    print(f"  RMS tracking error/joint [deg] = {np.round(_rms_tracking(d), 3)}")

    fig.tight_layout()
    out = os.path.join(os.path.dirname(csv), f"tam_debug_{tag}.png")
    fig.savefig(out, dpi=110)
    print(f"  saved {out}")
    plt.show()


def compare(path_a: str, path_b: str) -> None:
    """Overlay two runs (typically tam vs notam). Time bases are independent, so
    signals are plotted from t=0; the RMS tracking-error table is the robust
    quantitative comparison."""
    runs = [_load(path_a), _load(path_b)]
    colors = {"tam": "tab:blue", "notam": "tab:orange"}

    fig, ax = plt.subplots(3, 3, figsize=(18, 11))
    ax = ax.ravel()
    for j in range(7):
        for d, tag, _ in runs:
            t = d["t"] - d["t"][0]
            err = np.rad2deg(d[f"desired_q{j}"] - d[f"q{j}"])
            ax[j].plot(t, err, color=colors.get(tag), lw=0.8, label=tag)
        ax[j].set_title(f"tracking error j{j} [deg]")
        ax[j].legend(fontsize=8)
        ax[j].set_xlabel("t [s]")

    # RMS tracking-error bar comparison.
    axb = ax[7]
    width = 0.38
    x = np.arange(7)
    for i, (d, tag, _) in enumerate(runs):
        axb.bar(x + (i - 0.5) * width, _rms_tracking(d), width,
                color=colors.get(tag), label=tag)
    axb.set_xticks(x)
    axb.set_xticklabels([f"j{j}" for j in range(7)])
    axb.set_title("RMS tracking error / joint [deg]")
    axb.legend(fontsize=8)

    # Final-torque RMS comparison (proxy for effort / chatter).
    axc = ax[8]
    for i, (d, tag, _) in enumerate(runs):
        tf = _cols(d, "tau_final")
        axc.bar(x + (i - 0.5) * width, np.sqrt(np.mean(tf**2, axis=0)), width,
                color=colors.get(tag), label=tag)
    axc.set_xticks(x)
    axc.set_xticklabels([f"j{j}" for j in range(7)])
    axc.set_title("RMS final torque / joint [Nm]")
    axc.legend(fontsize=8)

    print("RMS tracking error / joint [deg]:")
    for d, tag, _ in runs:
        print(f"  {tag:>6}: {np.round(_rms_tracking(d), 3)}")

    fig.tight_layout()
    out = os.path.join(os.path.dirname(_find_ctrl_csv(path_a)), "tam_compare.png")
    fig.savefig(out, dpi=110)
    print(f"saved {out}")
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) == 2:
        diagnostic(sys.argv[1])
    elif len(sys.argv) == 3:
        compare(sys.argv[1], sys.argv[2])
    else:
        print(__doc__)
        sys.exit(1)
