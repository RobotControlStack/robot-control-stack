# RCS Flexiv Extension

This extension provides support for Flexiv arms such as the Rizon 4s and the Flexiv Grav gripper
in RCS, built on the Python bindings of the [Flexiv RDK](https://github.com/flexivrobotics/flexiv_rdk).

## Installation

```shell
pip install rcs-flexiv
```

For local development:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_flexiv
```

Complete the [first time setup](https://www.flexiv.com/software/rdk/manual/index.html#first-time-setup)
of the RDK once, release the E-stop and switch the robot into auto mode in Flexiv Elements before
connecting.

## Usage

```python
from rcs.envs.base import ControlMode
from rcs_flexiv.configs import DefaultRizon4SHardwareEnv

env_creator = DefaultRizon4SHardwareEnv()
env_creator.robot_sn = "Rizon4s-123456"
env_creator.tool_name = "Flexiv-GN01"  # tool created for the gripper in Flexiv Elements

cfg = env_creator.config()
cfg.control_mode = ControlMode.CARTESIAN_TQuat
cfg.robot_cfg.async_control = False

env = env_creator.create_env(cfg)
obs, info = env.reset()
```

## Control modes and stiffness

`FlexivConfig.control_mode` selects the RDK controller: `JOINT_POSITION`, `JOINT_IMPEDANCE`
(default) or `CARTESIAN_IMPEDANCE`. The impedance modes expose the controller gains as absolute
stiffness values (`joint_stiffness` in Nm/rad, `cartesian_stiffness` in N/m and Nm/rad, both within
`[0, nominal]`) and damping ratios within `[0.3, 0.8]`. `None` uses the robot's nominal values, and
`Flexiv.set_joint_impedance` / `Flexiv.set_cartesian_impedance` change them at runtime.

In the joint modes RCS's Cartesian control goes through its own IK like in simulation; in
`CARTESIAN_IMPEDANCE` the robot's Cartesian controller tracks the pose directly.

## Sync and async control

All RDK modes used here are non-real-time modes with an internal motion generator, so the robot
re-plans online whenever a new target arrives. With `async_control=True` the RCS setters return
immediately, with `async_control=False` they poll until the target is reached within tolerance or a
timeout hits. The same flag exists on `FlexivGripperConfig`. `move_home` always blocks.

## Notes

- The gripper is a device of the robot connection and is created from the `Flexiv` instance. Its
  device name is found in Flexiv Elements -> Settings -> Device, `Flexiv-GN01` for the Grav.
- The active tool in Flexiv Elements determines the gravity compensation, pass the gripper's tool
  as `tool_name`.
- The simulated counterpart is registered as `rcs/rizon4s` in core RCS and uses the same kinematics
  and gripper.

See `extensions/rcs_flexiv/README.md` for the full extension documentation and
`extensions/rcs_flexiv/src/rcs_flexiv/scripts/test_robot.py` for a bring-up script. For a maintained
example, see `examples/rizon4s/rizon4s_env_cartesian_control.py`, which moves the TCP forward and
backward in synchronous Cartesian mode in simulation or on hardware.
