# RCS Conventions

This page is the quick reference for coordinate frames, pose encodings, gripper semantics, and units used across RCS.
```text
Simple 3D sketch of the same right-handed frame (robot base is at the origin)

    +z (up)
    ^
    |
    o---> +y (left)
   /
  /
 +x (forward)
```

## At a glance

- **Quaternion order in RCS is `xyzw`** (`[qx, qy, qz, qw]`), matching Eigen's coefficient order.
- **MuJoCo free-joint `qpos` uses `wxyz`**, so direct MuJoCo state access needs an explicit reorder.
- **Robot/base frames are right-handed** with `x` forward, `y` left, and `z` up.
- **Gripper commands are normalized**: `0` means closed and `1` means open.
- **Euler angles are `roll`, `pitch`, `yaw`** around the `x`, `y`, and `z` axes.
- **Translations are in meters and angles are in radians** unless explicitly documented otherwise.

If you are working with multi-robot scene composition, also see the [Sim scene configuration guide](scene_configuration.md).

## Frames

### World frame

In simulation, scenes and free objects live in the global **world frame**.

Use world coordinates when you want to place something in the room itself, for example a cube on a table or a fixed camera in the lab.

The core robot API exposes explicit conversions between world and robot coordinates:

- `Robot.get_base_pose_in_world_coordinates()`
- `Robot.to_pose_in_world_coordinates(...)`
- `Robot.to_pose_in_robot_coordinates(...)`

### Robot base frame

Kinematics and low-level robot poses are expressed in the robot's **base frame**.

This is the frame assumed by the kinematics backends, for example in `src/rcs/Kinematics.cpp`, where `forward(...)` says the pose is assumed to be in the robot's coordinate frame.

When you interpret or command robot poses, the expected base-frame axis orientation is:

```text
x = forward
y = left
z = up
```

This is also the convention documented for Franka Quest teleoperation in `examples/teleop/README.md`.

### End-effector frame: `attachment_site`

Each robot config defines an `attachment_site`. This is the end-effector frame used by the kinematics stack.

Common examples in the repository are:

- `attachment_site_0` for FR3 / Panda
- `attachment_site` for UR5e / XArm7 / Rizon 4s
- `gripper` for SO101

If you are unsure which frame a robot uses, check its config or the relevant example scene in `python/rcs/envs/configs.py`.

### Tool frame: `tcp_offset`

`tcp_offset` is applied on top of the attachment site to define the actual tool center point (TCP) used by motion commands and IK.

In practice:

- `attachment_site` = frame coming from the robot model
- `tcp_offset` = extra transform from that frame to the tool you actually want to control

A typical example is a wrist-mounted gripper where the tool center point is slightly in front of the model's attachment site.

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

- `src/rcs/Pose.cpp`: `rotation_q()` returns `this->m_rotation.coeffs()`
- `python/tests/test_common.py` checks that the identity quaternion is `[0, 0, 0, 1]`

So the convention is:

```text
[qx, qy, qz, qw]
```

This matches Eigen's quaternion coefficient layout.

### `tquat`

`tquat` means translation plus quaternion and is used by the environment API.

The value layout is:

```text
[x, y, z, qx, qy, qz, qw]
```

This comes directly from `python/rcs/envs/base.py`, where `tquat` is built from `pose.translation()` followed by `pose.rotation_q()`.

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

These angles are rotations around the `x`, `y`, and `z` axes, respectively.

### Rotation vector / `rotvec`

Some hardware integrations, notably UR, also use a 6D rotation-vector pose:

```text
[x, y, z, rx, ry, rz]
```

You can see this in `extensions/rcs_ur5e/src/rcs_ur5e/hw.py`, where `common.RotVec(...).as_quaternion_vector()` is converted into an RCS `Pose`, and `Pose.rotvec()` is sent back to the robot.

## Gripper convention

Gripper actions and widths are normalized to the range `[0, 1]`.

The convention used across RCS is:

```text
0 = closed
1 = open
```

This is documented directly in multiple places:

- `python/rcs/envs/base.py`: `# 0 for closed, 1 for open (>=0.5 for open)`
- `src/sim/SimGripper.h`: `// normalized width of the gripper, 0 is closed, 1 is open`
- `extensions/rcs_fr3/src/hw/FrankaHand.h`: `// normalized width of the gripper, 0 is closed, 1 is open`

For binary grippers, the environment wrapper rounds and clips the command, and values `>= 0.5` are treated as open.

## Units

Unless a specific API says otherwise:

- translations, distances, and Cartesian offsets are in **meters**
- joint angles and Euler angles are in **radians**
- camera depth is stored as **metric depth scaled by 1000** in `uint16`, so values are effectively in **millimeters**

In practice, that means:

- moving `0.01` in `tquat[0]` means moving **1 cm** in `x`
- `np.pi / 2` means **90 degrees**
- a depth value of `723` means about **0.723 m**

The relevant camera path is:

- `python/rcs/camera/sim.py` converts the MuJoCo depth buffer to **meters** when `physical_units=True`
- `python/rcs/camera/interface.py` defines `BaseCameraSet.DEPTH_SCALE = 1000`
- the resulting metric depth is multiplied by that scale factor and stored as `uint16`

The same scaled-metric convention is also used by camera integrations such as RealSense and ZED.

## MuJoCo caveat: free-joint quaternions use `wxyz`

A common source of confusion is that **RCS uses `xyzw`**, but MuJoCo free-joint `qpos` stores the quaternion as **`wxyz`**.

RCS exposes both forms explicitly:

- `Pose.rotation_q()` returns `xyzw`
- `Pose.rotation_q_wxyz()` returns `wxyz`

You can see the `wxyz` form used when writing MuJoCo-facing state in files such as `python/rcs/sim/sim.py` and `python/rcs/envs/tasks.py`.

So:

- **RCS `Pose` / env API**: `xyzw`
- **MuJoCo free-joint state**: `wxyz`

If you manipulate MuJoCo state directly, convert between the two explicitly.

## Practical checklist

When something looks wrong, check these first:

1. Are you working in **world frame** or **robot/base frame**?
2. Does your robot/base frame follow **x forward, y left, z up**?
3. Is your end-effector frame the correct `attachment_site`?
4. Did you apply the right `tcp_offset`?
5. Are your quaternions **xyzw** in RCS?
6. Are you accidentally feeding **MuJoCo `wxyz`** into an RCS API?
7. Are your Euler angles ordered as **roll, pitch, yaw** around **x, y, z**?
8. Are your gripper commands using **0 = closed, 1 = open**?
9. Are your translations in **meters** and your angles in **radians**?
