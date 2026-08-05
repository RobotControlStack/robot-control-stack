# Franka Inference Example

This folder contains a hardware-oriented inference example for running a `vlagents` policy server against the Franka duo setup in [franka.py](franka.py) with configuration from [franka.json](franka.json).

## Policy Server

Before starting `franka.py`, make sure a `vlagents` policy server is already running. The example connects to a remote agent with:

- `vlagents_host`
- `vlagents_port`
- `vlagents_model`

The policy server setup and supported launch commands are documented in:

- [vlagents](https://github.com/RobotControlStack/vlagents)

Typical server startup looks like:

```shell
uv run python -m vlagents start-server lerobot --port 20000 --host 0.0.0.0 --kwargs '{"policy_name": "act", "checkpoint_path": "<path to pretrained_model>"}'
```

For other policies such as `pi05` or `xvla`, use the matching startup command from the `vlagents` README and make sure the values in `franka.json` point at that server.

## Config File

[franka.json](franka.json) is an example config, not a universal default. You should review and usually change these values before running inference:

- `vlagents_host`: Hostname or IP address where the policy server is running.
- `vlagents_port`: Port exposed by the policy server.
- `vlagents_model`: Agent id passed to `vlagents`, for example `lerobot`.
- `instruction`: Natural-language task instruction sent to the policy on reset.
- `robot_keys`: Robot names expected in each returned action dictionary and used to construct per-robot observations.
- `jpeg_encoding`: Whether observations are sent to the policy server using JPEG-compressed images.
- `on_same_machine`: Set this according to whether the policy server runs on the same machine as the control process.
- `image_size`: Client-side `(width, height)` resize applied before JPEG or shared-memory transport; defaults to `[224, 224]`. Set it to `null` to retain native resolution.
- `fps`: Control loop target frequency used by the local rate limiter.
- `record_path`: Output directory used when recording episodes.
- `n_action_steps`: Local action-chunk execution horizon. If `null`, the script requests and executes one action per control step. If set to a positive integer, it buffers up to that many actions from each policy response chunk.
- `max_rel_mov_joints`: Maximum allowed relative joint movement per step when running in joint control mode.
- `max_rel_mov_cart`: Maximum allowed relative Cartesian translation and rotation per step when running in Cartesian modes.

The current `franka.py` example also contains hardware-specific constants for robot IPs, camera serials, gripper serials, and frame mappings. Those live in the script itself, so update [franka.py](franka.py) if your hardware setup differs.

## Runtime Keys

When [franka.py](franka.py) is running, it waits for keyboard input on stdin. The active commands are:

- `e`: Start an episode without recording.
- `r`: Start an episode and begin recording to `record_path`.
- `s`: Mark the current episode as successful and reset the environment.
- `q`: Stop the current episode and reset the environment.
- `o`: Reload `franka.json`, reconnect the `vlagents` client, reset the environment, and clear any buffered actions.
- `x`: Exit the program.

## Observation And Action Mapping

The script translates RCS observations to the `vlagents` `Obs` format as follows:

- Camera frames are passed to `RemoteAgent` at native resolution; the client resizes them to `image_size` before JPEG or shared-memory transport.
- Each robot gets a `SingleObs` containing the shared camera set plus its own joints and gripper state.

Action chunks contain one action dictionary per environment step. For each robot, the script forwards `SingleAct.action` as the joint command and `SingleAct.gripper` as the gripper command. The action dictionary must include every configured `robot_key`.

## Running

After the policy server is up and `franka.json` is configured, run:

```shell
uv run python examples/inference/franka.py
```

If the policy server is unreachable, the script will keep retrying connection until it becomes available or you exit.
