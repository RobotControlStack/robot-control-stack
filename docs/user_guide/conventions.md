# Coordinate and Pose Conventions

This page collects the main 3D conventions used across RCS. It is meant as a quick reference when working with kinematics, environments, simulation objects, and teleoperation.

## Frames

### World frame

In simulation, scenes and objects live in a global **world frame**.

The core robot API exposes explicit conversions between world and robot coordinates:

- `Robot.get_base_pose_in_world_coordinates()`
- `Robot.to_pose_in_world_coordinates(...)`
- `Robot.to_pose_in_robot_coordinates(...)`

### Robot base frame

Kinematics and low-level robot poses are expressed in the robot's **base frame** (also called robot coordinates).

This is the frame assumed by the kinematics backends, for example in `extensions/rcs_robotics_library/src/pybind/RL.h`:

- `inverse(...)`: `pose is assumed to be in the robots coordinate frame`
- `forward(...)`: `pose is assumed to be in the robots coordinate frame`

The Franka hardware code uses the same convention and explicitly refers to the end-effector pose in the **base frame**.

### End-effector frame: `attachment_site`

Each robot config defines an `attachment_site`. This is the end-effector frame used by the kinematics stack.

Common examples in the repository are:

- `attachment_site_0` for FR3 / Panda
- `attachment_site` for UR5e / XArm7
- `gripper` in the SO101 examples

If you are unsure which frame a robot uses, check its config or the relevant example script.

### Tool frame: `tcp_offset`

`tcp_offset` is applied on top of the attachment site to define the actual tool center point (TCP) used by motion commands and IK.

In other words:

- `attachment_site` = default end-effector frame from the model
- `tcp_offset` = additional transform from that frame to the tool you want to control

## Pose representations

RCS uses several pose encodings. The important ones are:

### `Pose`

`rcs.common.Pose` is the main transform type. It supports construction from:

- translation only
- quaternion + translation
- `RPY` + translation
- rotation matrix + translation

### Quaternion order

Within RCS, quaternions are stored in **xyzw** order.

This is visible in two places:

- `python/tests/test_common.py` checks that the identity quaternion is `[0, 0, 0, 1]`
- `python/rcs/envs/sim.py` comments `rotation_q()` as `# xyzw format`

So the convention is:

```text
[qx, qy, qz, qw]
```

### `tquat`

`tquat` means translation plus quaternion and is used by the environment API.

The value layout is:

```text
[x, y, z, qx, qy, qz, qw]
```

This follows directly from `python/rcs/envs/base.py`, where `tquat` is built as:

```python
np.concatenate([
    pose.translation(),
    pose.rotation_q(),
])
```

### `xyzrpy`

`xyzrpy` is the translation plus roll-pitch-yaw representation used by the environment API.

The value layout is:

```text
[x, y, z, roll, pitch, yaw]
```

The `RPY` type in `python/rcs/_core/common.pyi` exposes the fields in exactly that order:

- `roll`
- `pitch`
- `yaw`

RCS examples and environment limits use `np.deg2rad(...)`, so angles are expected in **radians** unless explicitly documented otherwise.

### Rotation vector / `rotvec`

Some hardware integrations, notably UR, also use a 6D rotation-vector pose:

```text
[x, y, z, rx, ry, rz]
```

You can see this in `extensions/rcs_ur5e/src/rcs_ur5e/hw.py`, where `common.RotVec(...).as_quaternion_vector()` is converted into an RCS `Pose`, and `Pose.rotvec()` is sent back to the robot.

## MuJoCo caveat: free-joint quaternions use `wxyz`

A common source of confusion is that **RCS uses `xyzw`**, but MuJoCo free-joint `qpos` stores the quaternion as **`wxyz`**.

RCS already handles this conversion where needed. For example, `python/rcs/envs/sim.py` explicitly reorders the quaternion when writing directly into MuJoCo joint state:

```python
quat = self.init_object_pose.rotation_q()  # xyzw format
...
[self.x, self.y, self.z, quat[3], quat[0], quat[1], quat[2]]
```

So:

- **RCS `Pose` / env API**: `xyzw`
- **MuJoCo free-joint `qpos`**: `wxyz`

If you manipulate MuJoCo state directly, convert between the two explicitly.

## Axis convention

RCS generally works with robot-local base coordinates, and the standard Franka teleoperation setup documents the expected axis orientation as:

```text
x = front
y = left
z = up
```

This comes from `examples/teleop/README.md`, which also notes that the Quest alignment step should match the robot's base frame.

If you are integrating a new robot, make sure its model, `attachment_site`, and teleop calibration all agree on the same base-frame orientation.

## Practical checklist

When something looks wrong, check these first:

1. Are you working in **world frame** or **robot/base frame**?
2. Is your end-effector frame the correct `attachment_site`?
3. Did you apply the right `tcp_offset`?
4. Are your quaternions **xyzw** in RCS?
5. Are you accidentally feeding **MuJoCo `wxyz`** into an RCS API?
6. Are your RPY values in **radians**?
7. Does your teleop or calibration frame match the robot base axes?
