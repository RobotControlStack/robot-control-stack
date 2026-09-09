"""Standalone C++ TAM adaptor self-test.

Feeds the golden static inputs (adaptor window + latent) through the C++ adaptor
MLP (`Franka.tam_forward_test`) and prints the residual, so it can be compared
against the Python reference output. Does NOT move the robot (it only builds a
Franka to reach the C++ method; the test path never commands torques).

Inputs (paths overridable via env vars):
  GOLDEN_INPUT=/home/tobi/Downloads/golden_input.npz   (adaptor_q/dq/tau_model, ...)
  Z_FINAL=/home/tobi/Downloads/z_final.npz             (z_final latent [7,64])
  ROBOT_IP=192.168.1.12

Run:
  python examples/inference/tam_cpp_test.py
"""

import os

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

import numpy as np

from rcs_fr3._core.hw import Franka, FR3Config

GOLDEN_INPUT = os.environ.get("GOLDEN_INPUT", "/home/tobi/Downloads/golden_input.npz")
Z_FINAL = os.environ.get("Z_FINAL", "/home/tobi/Downloads/z_final.npz")
ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.1.12")


def main() -> None:
    gi = np.load(GOLDEN_INPUT, allow_pickle=True)
    z = np.load(Z_FINAL, allow_pickle=True)

    # Adaptor window: q/dq are joint pos/vel; tau_model is already model-space
    # (row -1 = pre-TAM desired, rows :-1 = previously applied). So pass it as
    # tau_cmd with gravity = 0 (the C++ builds tau_model = tau_cmd + gravity).
    q = np.ascontiguousarray(gi["adaptor_q"], dtype=np.float64)
    qd = np.ascontiguousarray(gi["adaptor_dq"], dtype=np.float64)
    tau_model = np.ascontiguousarray(gi["adaptor_tau_model"], dtype=np.float64)
    gravity0 = np.zeros_like(tau_model)
    latent = np.ascontiguousarray(z["z_final"], dtype=np.float64).reshape(-1)

    print(f"golden window: q{q.shape} dq{qd.shape} tau_model{tau_model.shape}")
    print(f"latent: z_final{z['z_final'].shape} -> flat {latent.shape}")

    # Load the TAM adaptor weights (same default checkpoint the golden used:
    # dagger_applied_8850) into the C++ model.
    from simadaptor.deploy.history_runtime import RealTimeHistoryAdaptor

    rt = RealTimeHistoryAdaptor.from_checkpoint()
    weight = np.frombuffer(rt.adaptor_weight_bytes(), dtype=np.uint8).astype(np.float64)

    robot = Franka(FR3Config(ip=ROBOT_IP, ignore_realtime=True))
    robot.set_tam_mlp_weight(weight)

    T = robot.tam_history_steps()
    ld = robot.tam_latent_dim()
    print(f"C++ model: history_steps={T}, expected latent_dim={ld}")
    if latent.shape[0] != ld:
        print(f"WARNING: latent dim {latent.shape[0]} != model expected {ld}")

    print("\n--- running C++ tam_forward_test (real latent) ---")
    out = np.asarray(robot.tam_forward_test(q, qd, tau_model, gravity0, latent))

    print("\n--- running C++ tam_forward_test (ZERO latent) ---")
    out0 = np.asarray(robot.tam_forward_test(q, qd, tau_model, gravity0, np.zeros_like(latent)))

    # Authoritative reference (from tam_golden_check.py run):
    ref = np.array([-1.912711978, 0.07038974762, 2.086881399, -5.817387104,
                    -0.04524916410, -3.002661228, 0.1139977872])
    ref0 = np.array([0.09806400537, 0.1193237305, 0.09036725760, 0.09368467331,
                     0.1607908607, 0.1124634296, 0.04124310613])

    def cmp(tag, c, r):
        print(f"\n[{tag}]")
        print("  cpp:", np.array2string(c, precision=6, floatmode="fixed"))
        print("  ref:", np.array2string(r, precision=6, floatmode="fixed"))
        print("  |cpp-ref|:", np.array2string(np.abs(c - r), precision=6, floatmode="fixed"))
        print(f"  max|cpp-ref| = {np.abs(c - r).max():.4e}")

    cmp("real latent", out, ref)
    cmp("ZERO latent (isolates base MLP from conditioning)", out0, ref0)

    # If the golden file also carries a reference output, compare.
    for key in ("adaptor_delta_tau", "delta_tau", "residual", "tam_output"):
        if key in gi.files:
            ref = np.asarray(gi[key], dtype=np.float64).reshape(-1)
            diff = np.asarray(out) - ref
            print(f"\nreference '{key}': {np.array2string(ref, precision=9, floatmode='fixed')}")
            print(f"max |C++ - ref| = {np.abs(diff).max():.3e}")
            break
    else:
        print("\n(no reference output key in golden_input; compare the printed "
              "value against your Python implementation)")


if __name__ == "__main__":
    main()
