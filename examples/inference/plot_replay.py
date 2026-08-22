"""Plot a replay run: dataset-recorded vs replay-measured joint motion.

Usage:
    python plot_replay.py tam_logs/<timestamp>

Reads <dir>/replay_frames.csv (written by franka_replay.py) and overlays, per
joint: the sent target (q_sent), what the robot measured during replay (q_meas),
and what the dataset originally recorded (q_ds). If q_meas tracks q_ds, the
execution reproduces the dataset; divergence localizes the sim2real gap.
"""

import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


def C(d, p):
    return np.stack([d[f"{p}{j}"] for j in range(7)], axis=1)


def _fk_xyz(Q):
    """Forward-kinematics FLANGE positions for an [N,7] joint array, or None if
    the rcs kinematics model is unavailable.

    The dataset EE convention is the flange (no gripper offset), and the robot
    control also runs with tcp_offset = identity (franka_tam), so we FK the
    flange here to compare like-for-like. (Adding the Robotiq 2F-85 TCP offset
    changes EE-Z by <1 mm anyway, since Z error is dominated by flange position.)
    """
    try:
        import rcs
        from rcs import common as rc

        robot = rcs.ROBOTS[rc.RobotType.FR3]
        pin = rc.Pin(robot.mjcf_model_path, robot.attachment_site, False)
        flange = rc.Pose()  # identity: flange frame, matches dataset + control
        return np.array([pin.forward(q, flange).translation() for q in Q])
    except Exception as e:  # noqa: BLE001
        print(f"[plot_replay] FK unavailable ({e}); skipping EE comparison")
        return None


def main(log_dir: str) -> None:
    csv = log_dir if os.path.isfile(log_dir) else os.path.join(log_dir, "replay_frames.csv")
    if not os.path.isfile(csv):
        hits = glob.glob(os.path.join(log_dir, "replay_frames.csv"))
        csv = hits[0] if hits else csv
    d = np.genfromtxt(csv, delimiter=",", names=True)
    t = d["t_ds"] - d["t_ds"][0]
    q_sent, q_meas, q_ds = C(d, "q_sent"), C(d, "q_meas"), C(d, "q_ds")

    # Replay fidelity = how well the robot reproduced the dataset's own motion.
    rms_track = np.rad2deg(np.sqrt(np.mean((q_meas - q_ds) ** 2, axis=0)))
    print(f"replay vs dataset RMS deviation / joint [deg] = {np.round(rms_track, 2).tolist()}")
    print(f"max |q_meas - q_ds| / joint [deg] = {np.round(np.rad2deg(np.abs(q_meas - q_ds).max(0)), 2).tolist()}")

    fig, axes = plt.subplots(4, 2, figsize=(14, 13), sharex=True)
    axes = axes.ravel()
    for j in range(7):
        ax = axes[j]
        ax.plot(t, np.rad2deg(q_ds[:, j]), color="tab:blue", lw=1.6, label="dataset (recorded)")
        ax.plot(t, np.rad2deg(q_meas[:, j]), color="tab:red", lw=1.2, label="replay (measured)")
        ax.plot(t, np.rad2deg(q_sent[:, j]), color="tab:green", lw=0.7, ls="--", alpha=0.7, label="sent target")
        ax.set_title(f"joint {j}   (replay vs dataset RMS {rms_track[j]:.2f}°)")
        ax.set_ylabel("deg")
        ax.grid(alpha=0.3)
        if j == 0:
            ax.legend(fontsize=8)
    # gripper
    axes[7].plot(t, d["gripper_sent"], color="tab:purple", lw=1.2)
    axes[7].set_title("gripper sent")
    axes[7].set_ylim(-0.1, 1.1)
    axes[7].grid(alpha=0.3)
    for ax in (axes[6], axes[7]):
        ax.set_xlabel("t [s]")

    fig.suptitle("Replay: dataset vs measured joint motion", y=1.0)
    fig.tight_layout()
    out = os.path.join(os.path.dirname(csv), "replay_compare.png")
    fig.savefig(out, dpi=110)
    print(f"saved {out}")

    # --- EE / Cartesian comparison (FK of measured vs dataset joints) ----------
    # Small joint errors amplify through the kinematics; EE-Z is grasp-relevant.
    em, ed = _fk_xyz(q_meas), _fk_xyz(q_ds)
    if em is not None and ed is not None:
        derr = (em - ed) * 1000.0  # mm
        rms = np.sqrt(np.mean(derr ** 2, axis=0))
        mx = np.abs(derr).max(axis=0)
        print("EE replay vs dataset  RMS [mm]  x/y/z =", np.round(rms, 1).tolist())
        print("EE replay vs dataset  max [mm]  x/y/z =", np.round(mx, 1).tolist())
        print(f"EE-Z mean(replay - dataset) = {derr[:, 2].mean():.1f} mm (>0 = replay higher)")

        # EE x/y/z trajectories over time (dataset vs replay) + the error panel.
        fig2, a2 = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
        for k, lbl in enumerate(("x", "y", "z")):
            a2[k].plot(t, ed[:, k] * 1000, color="tab:blue", lw=1.5, label="dataset (sim)")
            a2[k].plot(t, em[:, k] * 1000, color="tab:red", lw=1.2, label="replay (measured)")
            a2[k].set_ylabel(f"EE {lbl} [mm]"); a2[k].grid(alpha=0.3)
            a2[k].set_title(f"EE {lbl}: replay vs dataset  (RMS {rms[k]:.1f} mm, max {mx[k]:.1f} mm)")
            if k == 0:
                a2[k].legend(fontsize=8)
        for k, lbl, c in [(0, "x", "tab:green"), (1, "y", "tab:orange"), (2, "z", "tab:red")]:
            a2[3].plot(t, derr[:, k], c, lw=1.0, label=f"{lbl} err")
        a2[3].axhline(0, color="k", lw=0.6)
        a2[3].set_ylabel("replay - dataset [mm]"); a2[3].set_xlabel("t [s]")
        a2[3].set_title("EE position error (replay - dataset)"); a2[3].grid(alpha=0.3); a2[3].legend(fontsize=8)
        fig2.suptitle("Replay: EE trajectory vs dataset (flange frame)", y=1.0)
        fig2.tight_layout()
        out2 = os.path.join(os.path.dirname(csv), "replay_ee.png")
        fig2.savefig(out2, dpi=110)
        print(f"saved {out2}")

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
