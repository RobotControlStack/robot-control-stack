# Lazy camera rendering from state snapshots

Status: implemented (`StateSnapshotter` in `src/sim/sim.h`, `SimCameraSet` in
`python/rcs/camera/sim.py`); the optional snapshot ring (item 6) is not.

## Motivation

Simulation cameras used to be rendered either when their frames were accessed
(`render_on_demand=True`) or at a fixed camera frame rate while the simulation
steps. The fixed rate is realistic: a real camera produces its last frame at
some point *before* the controller reads it, so the observation lags behind the
current simulation state by up to one camera period. It is also wasteful,
because every frame between two accesses is rendered and thrown away.

The goal is to keep the realistic timing but only ever render the frames that
are actually accessed.

## Design

1. **Camera ticks in C++.** The camera frame rates are made known to the C++
   `Sim` where the stepping loop lives (`Sim::register_state_snapshot(rate)` or
   similar). While stepping, whenever `time - last_snapshot_time >= 1 / rate`
   has been *crossed* (not "equals": the physics timestep rarely divides the
   camera period), the current simulation state is copied with
   `mj_getState(mjSTATE_INTEGRATION)` together with the actual simulation time
   of that step. This covers qpos/qvel/act/ctrl and mocap bodies, so
   teleoperated and mocap-driven scenes render correctly. The copy happens under
   a lock so that `async_control` (stepping in a C++ thread) is safe. No
   rendering happens in C++.
2. **One snapshot per frame rate, not per camera.** Cameras with the same frame
   rate share a snapshot; cameras with different rates get their own. The camera
   set registers the rates it needs when it is created and reads the latest
   `(time, state)` back by rate. Frames of one `FrameSet` may therefore carry
   different timestamps, like real cameras do (`Frame.avg_timestamp` /
   per-camera `DataFrame.timestamp` already support this).
3. **Rendering in Python at access time.** `get_latest_frames()` fetches the
   latest snapshot per rate. If it was already rendered (same snapshot
   timestamp), the cached frames are returned. Otherwise the state is written
   into a scratch `MjData` (allocated once) with `mj_setState`, followed by
   `mj_kinematics` and `mj_camlight` (plus `mj_tendon` / `mj_flex` if the model
   has tendons or flexes; no collision detection, so this is well below a
   millisecond), and the cameras are rendered from the scratch data. Exactly one
   render per camera per access.
4. **`render_current` flag.** `SimCameraSet(render_current=True)` skips the
   snapshots and renders the live state at the time of access (today's
   `render_on_demand=True` behaviour). The default is the realistic snapshot
   behaviour. `render_on_demand` stays accepted as a deprecated alias for one
   release.
5. **Before the first tick.** If frames are accessed before any snapshot exists
   (e.g. right after `reset()` at `t = 0`), the current state is rendered so
   that reset observations keep working.
6. **Optional: ring of snapshots.** Keeping the last N snapshots per rate makes
   `get_timestamp_frames(ts)` lazy and exact as well: the snapshot matching
   `ts` is rendered on request instead of hoping that a frame was rendered at
   that time.

## Cost

- Per tick: one `mj_getState` (a few KB, microseconds).
- Per access: `mj_setState` + kinematics (~0.1-0.5 ms for an arm scene) + one
  render per camera.
- Compared to rendering at a fixed rate this removes all renders of frames that
  are never read; e.g. a 30 Hz camera read by a 10 Hz controller renders 3x less.

## Caveats

- Anything a renderer derives from `MjData` beyond kinematics (e.g. contact
  visualization) is not available in the scratch data. Irrelevant for camera
  images.
- Depth rendering with the Filament backend still uses the classic renderer; it
  simply renders from the same scratch data.

## Rough size

~50 lines of C++ (snapshotter using the existing `Callback` bookkeeping in
`sim.cpp`, plus pybind), ~80 lines in `python/rcs/camera/sim.py`, tests.
