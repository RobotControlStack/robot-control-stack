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

- **Viewer**: plots of joints, gripper, TCP, actions and the controller rate signals, camera playback,
  relabel (successful / unsuccessful) and delete of episodes.

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
The IP of the PC in that network is set in the web page (*Settings*, default `10.42.0.1`).

### 4. Start the interface

```shell
rcs-webui serve --data-root ~/rcs_data          # optional: --port 8080 --host 0.0.0.0
```

Open `http://<pc>:8080`. No env runs yet. Everything is configured in the page and kept in
`<data-root>/.rcs_webui/settings.json`:

- *Settings*: mode (collect / eval), operator (Meta Quest or web buttons only), Quest network IP, swap left /
  right, MuJoCo viewer (sim), the rate of default configs, config directories (their `*.json` files are offered
  as configs) and env packages (modules that register more envs, default `duobench.tasks`). Mode, operator,
  Quest and viewer settings are used at env start and can only be changed while no env runs.
- *Auto reset* (on by default): when the env ends the episode itself, it is stored and the env is reset: as
  successful if the task check reports success (`terminated` with `info["success"]`, e.g. DuoBench tasks), as
  unsuccessful for any other termination or truncation. Off: the env's end is only shown in the status and the
  episode keeps recording until `B` / `Y`. Robot errors always end the episode and reset the env.
- *On robot error*: `end episode` (default) stores the episode as unsuccessful (`truncated=True` and the message in
  `info.<robot>.robot_error` in the last row) and resets the env. `recover and continue` clears the error
  (`FR3HW.recover()`, libfranka automatic error recovery), the robot stays where it stopped and the episode keeps
  recording; the row of the error keeps `truncated=True` and the message, so converters can cut there. The Quest
  origin is reset: re-press the trigger to continue from where the robot is. If recovery fails, the episode ends.
- *Consolidate files on stop* (default on): when the env is stopped or restarted, the session's batch files are
  merged into files of about 512 MB (row groups of about 128 MB) in the background; relabel / delete of that session
  wait until it is done.
- *Env config*: pick a config, it is shown complete and editable, then *start env*. The list has the running
  env, the config directories, previous recordings and the default configs of the code (hardware: `fr3`,
  `fr3_duo`, `fr3_dual`, `droid`; simulation: `rcs/*` and e.g. `duobench/*` when DuoBench is installed).
  Changes against the picked config are marked in the gutter like in VS Code (green added, blue changed, red
  wedge deleted); click a mark to see the old and new lines and revert that change. Each start stores the
  picked config's id and the changed fields (`source.base` / `source.changed` in `session.json`).

The Quest operator is created with every env start: on hardware the Quest app gets an empty scene, in sim the
MuJoCo scene is streamed to the Quest. A config file holds the env creator and the serialized config
(`{"creator": ..., "config": ...}`, see `rcs-webui dump-config <env>`); a file without creator (e.g. an
`env_config.json`) gets the generic creator of its config type (`rcs_fr3.creators:RCSFR3MultiConfigEnvCreator`
for FR3 hardware, `rcs_webui.envs:ConfigSimEnvCreator` for sim). `robot_platform` of hardware robot configs is
set to `HARDWARE` (the C++ default `SIMULATION` is never read on hardware, the env class decides).

Try it in simulation first: pick e.g. `rcs/fr3` or `duobench/pick_place_red_on_green` (DuoBench branch
`juelg/single-arm`, a single FR3 with the DROID cameras `front_rgb` and `gripper_rgb`), set the operator to
*none* to use only the web buttons.

### 5. On the robot PC (*untested on hardware*)

1. Unlock the FR3 joints and activate FCI in Desk.
2. Put the config of your setup into a config directory, e.g. start from the default
   (`rcs-webui dump-config droid > ~/rcs_configs/droid.json`) and set robot IP, gripper and cameras with their
   serial numbers, or edit it in the page and start it there.
3. Hold the index trigger of the right controller to move the robot, the hand trigger closes the gripper.
   Press `A` to start recording, `B` (successful) or `Y` (unsuccessful) to end the episode, the env is reset
   (the robot moves home) after each episode. Ctrl+C stops the interface after flushing the recording, a second
   Ctrl+C exits immediately.

The control rate of the page (15 Hz by default) is the rate of every env it starts, whatever the config says: the
env's `frequency` (hardware: rate limited steps; sim: `sim_cfg.frequency`) and the robots' `policy_rate`, i.e. the
controller's interpolation window. Default configs are also adapted for recording:
`wrapper_cfg.record_control_states=True`, async control, and in collect mode the action space of the Quest (`tquat`
relative to the configured origin). Otherwise config files and previous recordings are used as stored.

### 6. Where the data lands

```text
<data-root>/
  2026-11-15_13-42-36_fr3_collect/      one folder per env (re)start: <date>_<time>_<env>_<mode>
    session.json                        creator, mode, instruction, frequency, how the config was obtained
    env_config.json                     the complete env config (offered again under previous recordings)
    git.json                            commit, branch, remote, untracked files of each git repo in use
    rcs.diff                            uncommitted changes (git diff HEAD) of that repo
    data/<date>_part-<n>-<id>.parquet   the recording, one or more files per episode
```

Read it with pyarrow or duckdb, e.g.
`duckdb -c "select uuid, count(*), bool_or(success) from read_parquet('<session>/data/*.parquet') group by uuid"`.
The 2D arrays (e.g. `info.right.control_states.q`, shape `N x 7`) are stored flattened, their shape is in the
column with the `_shape` suffix: `np.asarray(row["q"]).reshape(row["q_shape"])`.

### 7. Start the env with another config

Pick it in *Env config* and click *restart env*: the running env is closed (a running episode is stored as
unsuccessful) and a new data folder is created. *stop env* closes it without starting another one.

### 8. Look at the data

The *Data* tab lists the sessions and their episodes. Click an episode for the camera playback and the plots,
relabel it with the check / cross buttons or delete it (both rewrite the parquet files of that episode; the
episode that is being recorded can not be changed). `rcs-webui view --data-root ~/rcs_data` starts only the
viewer, e.g. on another machine.

### 9. Evaluate a policy

```shell
pip install -ve "extensions/rcs_webui[eval]"     # VLAgents client (only for the vlagents client)
```

Set the mode to *eval* in *Settings* (while no env runs), choose the client, host and port in *Policy server*
and start an env with a config that matches the policy. To try it without robot and server: client `mock`,
config `rcs/fr3`, operator *none*.

The env config must match the action space of the policy (VLAgents: absolute joints, `control_mode JOINTS`,
`relative_to NONE`); eval does not adapt default configs to the Quest. `A` starts a rollout, `B` / `Y` label it.
Holding the Quest trigger takes over (human intervention, only when the env uses the Quest action space). After
releasing the trigger the robot holds the pose the operator left it in and the human stays in control until `X`
(Quest or web button) hands it back to the policy, which continues with a fresh chunk from the current state. In the web page the policy server (client, host, port,
actions per chunk, RTC delay, client options) can be switched at any time without restarting the interface.

Every step of an episode has an `annotations` column: `controller` (who commanded the robot: `human` incl.
interventions, `learned_policy`, `scripted_policy`, `agent`, `none`; enum `rcs_webui.policy.Controller`), `source`
(`policy`, `intervention`, `teleop`, `idle`), per robot `engaged.<robot>` (Quest trigger held: the operator
commands that robot), `joint_speed.<robot>` (rad/s, max over the joints between the observations before and after
the step) and `moving.<robot>` (`joint_speed` above 0.0075 rad/s). `engaged == false` with `moving == false` is
operator repositioning (trigger released, robot still); `chunk_id`,
`chunk_index`, `request_id`, `request_sent`, `response_received` (host clock s), `switch_step` (episode step at
which the chunk took over) and `waited` (s the loop waited for the server in that step).

#### Policy server clients

A client implements `rcs_webui.policy.PolicyClient` (`connect`, `reset(instruction)`, `infer(ChunkRequest) ->
ActionChunk`, `close`, `capabilities`). It is selected in *Policy server* by built-in name (`vlagents`, `mock`), by an
entry point of the group `rcs_webui.policy_clients` of an installed package, or as `my_package.module:MyClient`.
Clients of other projects live in those projects, RCS only ships the VLAgents client (untested against a running
VLAgents server, it follows `examples/inference/franka.py`) and the mock.

#### Real-time chunking (off by default)

With `--rtc-delay d` (only for clients with `capabilities.realtime_chunking`) the next chunk is requested `d` steps
before the switch-over step (after `--execute-steps` actions of the current chunk, default the whole chunk), from a
worker thread while the robot keeps executing. The request carries the observation, the committed prefix (the `d`
raw actions executed meanwhile, `prefix`, with `prefix_chunk_id` / `prefix_start`) and the rest of the current
chunk (`guidance`, the switch-over part to inpaint). The returned chunk is aligned to the query step; the loop
switches to it at the switch-over step at index `d`, or as soon as it arrives if it is late (at the aligned index),
and waits only if the current chunk is used up.

### Troubleshooting

- `simpub is not installed`: install SimPublisher (step 3) or set the operator to *none* in *Settings*.
- `[Quest Reader] packets empty` in the log: the Quest app is not running or not in the same network,
  check the Quest IP in *Settings* and the name `RCSNode`.
- The robot does not follow the controller: it only moves while the index trigger is held.
- A reflex ends the episode: the status shows `episode ended by the env`, the episode is stored with
  `truncated=True` and the message in `info.<robot>.robot_error`; the next reset recovers the robot.
- Rendering errors in headless simulation: `export MUJOCO_GL=egl`.
