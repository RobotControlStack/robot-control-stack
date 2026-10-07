# Sim Scene Configuration

This page explains the simulation scene API in `python/rcs/envs/scenes.py` and shows how the example scene configs in `python/rcs/envs/configs.py` fit together.

If you are new to the frame conventions first, read [RCS Conventions](conventions.md).

```text
world frame
└── root frame
    ├── all composed scene objects live here
    └── shared base frame
        (same kinematic node as root frame)
        (common robot-coordinate frame)
        ├── i-th robot base
        │   └── i-th robot attachment_site
        ├── j-th robot base
        │   └── j-th robot attachment_site
        └── ...
```

## The simple mental model

When building an RCS sim scene, it helps to think about five frames:

1. **World frame**
   - The global MuJoCo frame.
   - This is the outermost reference for the whole scene.

2. **Root frame**
   - The scene-local frame for the composed robot setup.
   - In the usual composed setup, all robot-scene objects are placed from here.

3. **Shared base frame**
   - The common coordinate frame used for all robot actions and observations.
   - It is attached to the same kinematic node as the root frame, but represents the common robot coordinate convention.

4. **i-th robot base frame**
   - The base frame of one specific robot.
   - Low-level kinematics and Cartesian commands are expressed here.

5. **i-th robot `attachment_site`**
   - The end-effector mounting frame for that robot, before any optional `tcp_offset`.
   - Use this for wrist-mounted objects, cameras, and tools.

A good rule of thumb is:

- want the outer global reference -> **world frame**
- want the composed scene placement frame -> **root frame**
- want one common coordinate frame for all robots -> **shared base frame**
- want per-robot kinematics -> **i-th robot base frame**
- want wrist or tool mounting -> **i-th robot `attachment_site`**

If needed, extra `world_frame_objects` can still be placed directly in world coordinates.

## How the main placement frames work

For composed robot scenes, `SimEnvCreator.create_model()` combines three scene-placement transforms in this order:

```python
robot2world = root_frame_to_world * shared_base_frame_to_root_frame * robot_to_shared_base_frame[robot_name]
```

In simple terms:

- `root_frame_to_world` places the whole rig into the MuJoCo world
- `shared_base_frame_to_root_frame` defines the shared robot coordinate frame relative to the root frame
- `robot_to_shared_base_frame` places each robot relative to that shared robot frame

### Single-robot intuition

For a simple single-arm scene, all three can often be identity transforms:

```python
root_frame_to_world = rcs.common.Pose()
shared_base_frame_to_root_frame = rcs.common.Pose()
robot_to_shared_base_frame = {"robot": rcs.common.Pose()}
```

That means the robot base, shared base, root frame, and world origin all coincide.

### Dual-arm intuition

The FR3 duo example in `python/rcs/envs/configs.py` uses these frames more meaningfully:

- `root_frame_to_world` keeps the whole duo rig aligned with the world
- `shared_base_frame_to_root_frame` lifts the shared base to the duo mount height
- `robot_to_shared_base_frame` offsets the left and right robots from the center

That is why the dual-arm example can expose one common action frame while still placing each robot correctly.

## `SimEnvCreatorConfig` keys

This is the main top-level config for the scene API.

| Key | What it controls | Typical use |
| --- | --- | --- |
| `robot_cfgs` | Maps robot names to `SimRobotConfig` | Define one robot or multiple named robots such as `"left"` and `"right"` |
| `sim_cfg` | MuJoCo runtime settings (`SimConfig`) | Realtime, async control, frequency, convergence behavior |
| `control_mode` | Action representation | For example `ControlMode.CARTESIAN_TQuat` |
| `task_cfg` | Optional task-specific config | Add pick/place or other task logic |
| `scene` | Base scene XML path or scene key | Usually from `SCENE_PATHS[...]` |
| `gripper_cfgs` | Optional gripper config per robot | Add one gripper per robot |
| `camera_cfgs` | Optional camera config dictionary | Define resolution, type, and frame rate for named cameras |
| `max_relative_movement` | Relative action limit | Limit per-step Cartesian delta |
| `relative_to` | Relative action reference | Usually `RelativeTo.LAST_STEP` or `RelativeTo.NONE` |
| `robot_to_shared_base_frame` | Per-robot offset relative to the shared base frame | Multi-robot layouts |
| `add_gravcomp` | Add gravity compensation to the composed scene | Often useful for manipulation scenes |
| `wrapper_cfg` | Wrapper behavior flags | Binary gripper, home-on-reset, depth output |
| `headless` | GUI toggle | `True` for no GUI |
| `shared_base_frame_to_root_frame` | Offset from shared base frame to root frame | Move shared command origin inside the rig |
| `root_frame_to_world` | Offset from root frame to MuJoCo world | Place the whole setup in the room |
| `alternative_combined_robot_mjcf` | Use a pre-combined robot MJCF instead of composing robots one by one | Advanced custom scenes |
| `world_frame_objects` | Objects placed directly in world coordinates | Loose props, room-fixed assets |
| `root_frame_objects` | Objects placed in root-frame coordinates | Tables, mounts, fixtures that should move with the rig |
| `robot_frame_objects` | Objects attached in a robot attachment-site frame | Wrist mounts, end-effector payloads |
| `camera_adds` | Cameras to add to the scene | Fixed overhead cameras or wrist cameras |
| `gripper_offsets` | Pose offsets for mounted grippers | Align visual or tool frames |
| `_original_cfg` | Internal helper used after prefixing | Usually ignore this in user code |

## What usually lives inside the nested configs

The scene config mostly wires together three lower-level config types.

### `robot_cfgs: dict[str, SimRobotConfig]`

Each robot entry usually defines things such as:

- robot type
- kinematic model path
- `attachment_site`
- `tcp_offset`
- joint names and actuator names
- base link name
- degrees of freedom, joint limits, and `q_home`

The single-arm and dual-arm examples in `python/rcs/envs/configs.py` are good templates for this.

### `gripper_cfgs: dict[str, SimGripperConfig]`

Each gripper entry usually defines:

- gripper type
- gripper joint names and actuator name
- min/max width or actuator range
- collision geometry settings
- callback timing

### `camera_cfgs: dict[str, SimCameraConfig]`

Each camera entry usually defines:

- camera identifier
- camera type
- resolution
- frame rate

A useful pattern is:

- `camera_cfgs` defines the camera runtime properties
- `camera_adds` defines where that camera is placed in the scene

## The most important nested configs

### `WrapperConfig`

`WrapperConfig` controls behavior of the environment wrappers around the raw simulation.

| Key | Meaning |
| --- | --- |
| `binary_gripper` | If `True`, gripper commands are treated as open/close instead of continuous width |
| `home_on_reset` | If `True`, the robot returns home during reset |
| `include_depth` | If `True`, camera wrappers include depth images. These are metric depth values scaled by `BaseCameraSet.DEPTH_SCALE = 1000` and stored as `uint16`, so they are effectively in millimeters |

### `CameraAdderConfig`

`CameraAdderConfig` describes how a camera is added to the scene.

| Key | Meaning |
| --- | --- |
| `xml_path` | Optional camera XML asset to insert directly |
| `fovy` | Camera field of view, used when creating a camera directly |
| `offset` | Camera pose offset |
| `attachment_site` | Attachment site to use if mounted on a robot |
| `robot_name` | If set, mount the camera on that robot; otherwise add it as a scene camera |

The important frame detail is:

- if `robot_name` is **not** set, `offset` is interpreted in the **root frame** and then moved into world by `root_frame_to_world`
- if `robot_name` **is** set, `offset` is interpreted relative to that robot's **attachment site**

## Easy examples

### Minimal single-robot scene

This is the basic shape used by `EmptyWorldFR3` in `python/rcs/envs/configs.py`:

```python
cfg = SimEnvCreatorConfig(
    robot_cfgs={"robot": robot_cfg},
    sim_cfg=SimConfig(async_control=False, realtime=True, frequency=1),
    control_mode=ControlMode.CARTESIAN_TQuat,
    scene=SCENE_PATHS["empty_world"],
    gripper_cfgs={"robot": gripper_cfg},
    camera_cfgs={"bird_eye": bird_eye_cfg, "wrist": wrist_cfg},
    robot_to_shared_base_frame={"robot": rcs.common.Pose()},
    shared_base_frame_to_root_frame=rcs.common.Pose(),
    root_frame_to_world=rcs.common.Pose(),
)
```

What this means in plain language:

- there is one robot named `robot`
- it uses Cartesian `tquat` actions
- the base scene is the empty world
- there is one gripper on the robot
- there are two cameras
- all high-level frames start at the same origin

### Dual-arm scene

This is the important part of the `EmptyWorldFR3Duo` example:

```python
robot_cfgs = {"left": robot_cfg_left, "right": robot_cfg_right}

robot_to_shared_base_frame = {
    "left": DEFAULT_TRANSFORMS["FR3_DUOMOUNT_LEFT_ROBOT"],
    "right": DEFAULT_TRANSFORMS["FR3_DUOMOUNT_RIGHT_ROBOT"],
}

shared_base_frame_to_root_frame = DEFAULT_TRANSFORMS["FR3_DUOMOUNT_HEIGHT_OFFSET"]
root_frame_to_world = rcs.common.Pose()
```

In plain language:

- the shared base frame sits at the logical center of the duo setup
- the left and right robot bases are offset from that center
- the whole setup can still be moved together by changing `root_frame_to_world`

### Object placement: which dictionary should I use?

#### `world_frame_objects`

Use this when the object should stay fixed in the room.

```python
world_frame_objects = {
    "cube": (OBJECT_PATHS["green_cube"], rcs.common.Pose(translation=np.array([0.5, 0.0, 0.2]))),
}
```

Example meaning: place a cube at a fixed world position.

#### `root_frame_objects`

Use this when the object belongs to the rig and should move together with it.

```python
root_frame_objects = {
    "duo_mount": (OBJECT_PATHS["fr3_duo_mount"], DEFAULT_TRANSFORMS["FR3_DUOMOUNT_BASE"]),
}
```

Example meaning: the duo mount is part of the setup, not a free world object.

#### `robot_frame_objects`

Use this when the object should be attached to one robot's tool frame.

```python
robot_frame_objects = {
    "left": {
        "left_d405_mount": (
            OBJECT_PATHS["robotiq_d405_mount"],
            DEFAULT_TRANSFORMS["FR3_ROBOTIQ_WRIST_D405_MOUNT"],
        )
    }
}
```

Example meaning: attach a wrist mount to the left robot only.

## Camera depth units

When depth is enabled, the camera wrapper exposes depth images as scaled metric depth:

- sim depth is first converted to **meters** in `python/rcs/camera/sim.py`
- camera frames use `BaseCameraSet.DEPTH_SCALE = 1000`
- depth is then stored as `uint16`

So in practice:

- divide by `1000` to get **meters**
- or read the values directly as **millimeters**

Example:

- `depth[y, x] == 1500` means the point is about **1.5 m** away from the camera

## Rendering backend (Filament)

By default, simulation cameras and the GUI use MuJoCo's classic OpenGL renderer. MuJoCo >= 3.15 also
ships the physically based [Filament](https://github.com/google/filament) renderer and the new
MuJoCo Studio viewer. Switch to them with the `renderer` flag of `SimConfig`:

```python
from rcs.sim import RendererBackend, SimConfig

sim_cfg = SimConfig(async_control=False, realtime=False, renderer=RendererBackend.FILAMENT)
```

With this flag

- `SimCameraSet(...)` returns a `FilamentSimCameraSet`: color images are rendered with Filament in
  Python (frames are rendered when requested, a fixed camera frame rate is not supported). Filament does
  not expose a metric depth buffer through MuJoCo's Python API, so depth images are still rendered with
  the classic renderer and always use the metric units described above.
- `open_gui()` launches MuJoCo Studio instead of the passive `mujoco.viewer`. Studio runs in the GUI
  subprocess under a regular `python` interpreter (not `mjpython`, also on macOS).

Environment variables for fine tuning:

- `RCS_FILAMENT_GRAPHICS_API=opengl|vulkan` graphics API of the camera renderer (default `opengl`)
- `RCS_FILAMENT_SOFTWARE_RENDERING=1` force software rendering for the cameras
- `RCS_FILAMENT_GFX=opengl|vulkan|web|...` Studio graphics mode; `web` serves the viewer over http instead
  of opening a window

### Camera placement

#### Fixed scene camera

This pattern from `EmptyWorldFR3` adds an overhead camera:

```python
camera_adds = {
    "bird_eye": CameraAdderConfig(
        fovy=60.0,
        offset=rcs.common.Pose(
            translation=np.array([0.271, 0.0, 2.080]),
            quaternion=np.array([0.0060, -0.0060, -0.7067, 0.7074]),
        ),
    )
}
```

Because `robot_name` is not set, this pose is interpreted in the **root frame**.

#### Wrist camera

This pattern mounts a camera to a robot:

```python
camera_adds = {
    "wrist": CameraAdderConfig(
        fovy=60.0,
        offset=some_pose,
        robot_name="robot",
    )
}
```

Because `robot_name` is set, `offset` is interpreted relative to that robot's **attachment site**.

## Common mistakes

1. **Mixing up world and root frame**
   - If the whole rig should move together, use `root_frame_to_world` or `root_frame_objects`, not `world_frame_objects`.

2. **Using the wrong frame for camera offsets**
   - Scene cameras use root-frame offsets.
   - Robot-mounted cameras use attachment-site offsets.

3. **Forgetting matching camera names**
   - If a camera is added without `xml_path`, its name must also exist in `camera_cfgs`.

4. **Putting wrist-mounted assets into world objects**
   - Use `robot_frame_objects` for things that should follow the robot wrist.

5. **Using `alternative_combined_robot_mjcf` without the expected prefixes**
   - The docstring in `scenes.py` requires names like `robot{robot_name}`.

## A practical workflow

When building a new scene, this usually works well:

1. Start with one robot and identity transforms.
2. Add `root_frame_objects` for mounts or fixtures.
3. Add `world_frame_objects` only for room-fixed props.
4. Add `robot_frame_objects` for wrist payloads.
5. Add cameras with `camera_adds`.
6. Only then introduce non-trivial `shared_base_frame_to_root_frame` and `robot_to_shared_base_frame` offsets.

That order keeps the frame reasoning much easier.
