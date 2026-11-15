# RCS Web Interface

A web interface for data collection and evaluation with RCS. It starts and owns the gym env, reads the
Meta Quest and records every step of every episode into the RCS parquet format with the `StorageWrapper`.

- **Labelling on the Quest**: `A` starts an episode, `B` ends it as successful, `Y` as unsuccessful.
  The same buttons are in the web page.
- **Everything in one place**: observations, camera frames, actions, the info dict and all robot states of
  the low level controller since the last step (1 kHz on the FR3, every physics step in sim) are stored in
  the same parquet rows. A robot error (e.g. a reflex) ends the episode with `truncated=True` and the error
  message in `info.<robot>.robot_error`.
- **Reproducible**: every env (re)start creates a new data folder with the complete env config, the git
  commit and the git diff. A new env can be started from that stored config.

Status of this PR (draft): data collection with the Quest is implemented and tested in simulation.
Data viewer, relabel / delete, eval mode with the policy server switch follow in the same PR.

## Setup from scratch

Steps marked *untested on hardware* were only checked in simulation or with mocked hardware.

### 1. Repositories and versions

| what | where | version |
|------|-------|---------|
| RCS incl. this extension and `rcs_fr3` | `https://github.com/RobotControlStack/robot-control-stack` | branch `feat/webui-extension` (based on v0.7.4) |
| Quest app (IRIS) | `https://github.com/RobotControlStack/IRIS-Meta-Quest3/releases/tag/rcsv0.7.x` | `rcsv0.7.x` APK |
| SimPublisher (Python client of the Quest app) | `https://github.com/RobotControlStack/SimPublisher` | default branch |

Linux (Debian / Ubuntu LTS), Python 3.11, a real time kernel for the FR3 (see the libfranka docs).

### 2. Install RCS, the FR3 extension and this extension

```shell
git clone -b feat/webui-extension https://github.com/RobotControlStack/robot-control-stack.git
cd robot-control-stack
sudo apt install $(cat debian_deps.txt) $(cat extensions/rcs_fr3/debian_deps.txt)
python3.11 -m venv .venv && source .venv/bin/activate
pip install 'pip>=25.1'
pip install --group build_deps
pip install -ve . --no-build-isolation          # RCS core (C++ build, a few minutes)
pip install -ve extensions/rcs_fr3 --no-build-isolation   # FR3 (fetches and builds libfranka)
pip install -ve extensions/rcs_webui            # this extension, provides the rcs-webui command
```

Limit the parallel C++ build jobs on machines with little memory with `export CMAKE_BUILD_PARALLEL_LEVEL=8`.

Optional, depending on the setup:

```shell
pip install -ve extensions/rcs_robotiq2f85      # Robotiq 2F-85 gripper
pip install -ve extensions/rcs_realsense        # RealSense cameras
pip install -ve extensions/rcs_zed              # ZED cameras (needs the ZED SDK)
```

### 3. Meta Quest (operator)

Install the Quest app and SimPublisher as described in
[`examples/teleop/README.md`](../../examples/teleop/README.md) ("Teleoperation with Meta Quest 3"):
put PC and Quest into the same network, install the IRIS APK, `pip install` SimPublisher from source,
name the Quest `RCSNode` and align the coordinate frame with `examples/teleop/quest_align_frame.py`.
The IP of the PC in that network is passed with `--quest-addr` (default `10.42.0.1`).

### 4. Try it in simulation

```shell
rcs-webui envs                                   # envs that can be started
rcs-webui collect --env rcs/fr3 --data-root ~/rcs_data --instruction "pick up the cube"
# without a Quest, control episodes with the web buttons only:
rcs-webui collect --env rcs/fr3 --operator none --data-root ~/rcs_data
```

Open `http://<pc>:8080` in a browser. Add `--gui` for the MuJoCo viewer.

### 5. Start on the robot PC (*untested on hardware*)

1. Unlock the FR3 joints and activate FCI in Desk.
2. Get the default config of your setup, edit it (robot IP, gripper, cameras with their serial numbers) and
   start from it:

   ```shell
   rcs-webui dump-config fr3 > fr3.json          # single FR3 with Franka Hand, robot name "right"
   # or: fr3_duo (two FR3 with Robotiq), droid, fr3_dual
   rcs-webui collect --env fr3 --config fr3.json --data-root ~/rcs_data --quest-addr 10.42.0.1 \
       --instruction "pick up the red cube and place it in the bowl"
   ```

   Small changes can also be given directly: `--set robot_cfgs.right.ip=192.168.101.1`
   (dotted path into the JSON config, the value is parsed as JSON and taken as string otherwise).
3. Open `http://<robot-pc>:8080`. Hold the index trigger of the right controller to move the robot, the
   hand trigger closes the gripper. Press `A` to start recording, `B` (successful) or `Y` (unsuccessful)
   to end the episode, the env is reset (the robot moves home) after each episode.

Default configs are adapted for recording: `wrapper_cfg.record_control_states=True`, async control at
`--frequency` (default 15 Hz, also the controller interpolation window `policy_rate`), and the action space
of the Quest (`tquat` relative to the configured origin). A config file or a previous recording is used
exactly as stored.

### 6. Where the data lands

```text
<data-root>/
  2026-11-15_13-42-36_fr3_collect/      one folder per env (re)start: <date>_<time>_<env>_<mode>
    session.json                        creator, mode, instruction, frequency, how the config was obtained
    env_config.json                     the complete env config (load it with --from-recording)
    git.json                            commit, branch, remote, untracked files of each git repo in use
    rcs.diff                            uncommitted changes (git diff HEAD) of that repo
    data/<date>_part-<n>-<id>.parquet   the recording, one or more files per episode
```

Read it with pyarrow or duckdb, e.g.
`duckdb -c "select uuid, count(*), bool_or(success) from read_parquet('<session>/data/*.parquet') group by uuid"`.
The 2D arrays (e.g. `info.right.control_states.q`, shape `N x 7`) are stored flattened, their shape is in the
column with the `_shape` suffix: `np.asarray(row["q"]).reshape(row["q_shape"])`.

### 7. Restart the env with another config

In the web page: *Restart env*, start from the same config, the default config of another env or the
config of a previous recording, optionally with overrides as JSON (`{"sim_cfg.frequency": 20}`).
From the CLI: `rcs-webui collect --from-recording <data-root>/<session>`.
Every restart closes the running env (a running episode is stored as unsuccessful) and creates a new folder.

### Troubleshooting

- `simpub is not installed`: install SimPublisher (step 3) or start with `--operator none`.
- `[Quest Reader] packets empty` in the log: the Quest app is not running or not in the same network,
  check `--quest-addr` and the name `RCSNode`.
- The robot does not follow the controller: it only moves while the index trigger is held.
- A reflex ends the episode: the status shows `episode ended by the env`, the episode is stored with
  `truncated=True` and the message in `info.<robot>.robot_error`; the next reset recovers the robot.
- Rendering errors in headless simulation: `export MUJOCO_GL=egl`.
