# TAM (Torque Adaptation Module) integration

[TAM](https://github.com/Dongwon-Son/TAM) adds a learned residual to the
commanded joint torque at 1 kHz inside the RCS controller, conditioned on a
latent computed from the recent control history. One process, two rates:

- **1 kHz (C++ control thread)**: `Franka::tam_forward` runs a small MLP on
  q, dq, the last 8 commanded torques (gravity-included space) and the
  current latent, and adds the clipped residual to the controller torque.
- **~5 Hz (Python thread in `franka_tam.py`)**: `run_history_encoder` pulls
  the controller's history buffer, streams it through the TAM transformer
  encoder (JAX), and pushes each latent back via `set_tam_latent`.

## Setup

```shell
pip install -r requirements.txt   # includes torque-adaptation-module (JAX)
```

A CUDA-capable JAX is strongly recommended on the control machine — the
encoder is ~20x slower than real time on CPU:

```shell
pip install "jax[cuda12]"
```

## Run

Set `tam = True` on `InferenceConfig` (top of `franka_tam.py`; the file does
not read `franka.json`). Everything else resolves automatically on first run:

- the default checkpoint (DAgger-finetuned, applied-torque) downloads once
  from the TAM GitHub releases into `~/.cache/simadaptor` (override with
  `SIMADAPTOR_CACHE_DIR`), verified by SHA-256;
- the ideal-model MJCF is installed with the `torque-adaptation-module`
  package.

`tam_ckpt` / `tam_xml` on `InferenceConfig` override both.
`FrankaConfig.tam_residual_clip` (default 10/10/10/10/2/2/2 Nm) bounds the
per-joint residual; the residual also ramps in over 1 s whenever it
(re)activates.

## Operational notes

- The controller applies zero residual until the MLP weights and the first
  latent arrive. The first latent needs only ~0.5 s of control history (one
  400 ms encoder patch plus a poll); the estimate then keeps refining as
  context grows toward the 4 s attention window. Startup also pays a
  one-time JAX JIT warm-up of ~10-15 s before the encoder loop begins.
- Switching controller gains (`pd_mode`) restarts the control thread. The
  last latent is kept (it encodes plant properties, which a gain change
  does not alter), so the residual resumes right away and re-ramps over
  1 s; fresh latents resume ~0.5 s after the restart.
- Only applied-torque checkpoints are supported; `base_tam_fusion`
  checkpoints are rejected at startup.
- The residual MLP adds ~0.1-0.3 ms to every 1 kHz control tick, which
  leaves no deadline slack on a stock (non-PREEMPT_RT) kernel. With TAM
  enabled the example therefore sets `FrankaConfig.rt_priority = 80`: the
  control thread elevates itself to SCHED_FIFO, which works on stock
  kernels but requires an rtprio rlimit — one-time setup:

  ```shell
  echo "$USER - rtprio 99" | sudo tee -a /etc/security/limits.conf
  # then open a fresh login session and check: ulimit -r  ->  99
  ```

  Without the rlimit the controller prints a warning and stays on the
  normal scheduler; expect `communication_constraints_violation` aborts on
  a loaded machine in that state.
