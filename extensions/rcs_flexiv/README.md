# RCS Flexiv Extension

Support for Flexiv arms (Rizon 4s and other models supported by the RDK) and the Flexiv Grav
gripper in RCS, built on the Python bindings of the [Flexiv RDK](https://github.com/flexivrobotics/flexiv_rdk).

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_flexiv>

## Installation

Install from PyPI:

```shell
pip install rcs-flexiv
```

Install from a local checkout, using the local RCS checkout instead of the published `rcs-core`:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_flexiv
```

The `flexivrdk` package is pulled from PyPI and pinned to 1.9.2, which ships prebuilt wheels for
Linux, macOS and Windows and Python 3.10, 3.12 and 3.14.

The RDK version has to match the robot software version shown in Flexiv Elements, otherwise the
connection is refused as incompatible. RDK 1.9.x is the release line for the Rizon series, RDK 2.x
only supports the Enlight series and has a different API. If your robot runs another software
version, install the matching `flexivrdk` after this extension:

| robot software | `flexivrdk`  |
| -------------- | ------------ |
| v3.11.2        | 1.9.3        |
| v3.11.1        | 1.9.1, 1.9.2 |
| v3.11          | 1.9.0        |
| v3.10          | 1.8.x        |

## Robot setup

Go through the [first time setup](https://www.flexiv.com/software/rdk/manual/index.html#first-time-setup)
of the RDK manual once: the workstation needs a network connection to the robot controller and the
robot needs an RDK license. Before running RCS:

- Release the E-stop and put the robot into auto mode in Flexiv Elements.
- The serial number is passed as model name without spaces, a dash and the number, e.g.
  `Rizon4s-063650`.
- Create a tool for the mounted gripper in Flexiv Elements -> Settings -> Tool and pass its name as
  `tool_name`, so that gravity compensation accounts for the gripper. The Grav ships with a tool
  definition, check the tool list in Elements for its name.
- The gripper device name is listed in Flexiv Elements -> Settings -> Device, `Flexiv-GN01` for the
  Grav.
- Mounting angles and custom world frames configured in Flexiv Elements only affect the robot's own
  gravity compensation and the frame of the poses the RDK reports. RCS computes poses from the
  measured joints with its own model, so everything is in the base frame like in simulation. The
  transform between the two frames is estimated at connect (`Flexiv.world_from_base`) and used
  when commanding the robot's Cartesian controller.

## Control modes

`FlexivConfig.control_mode` selects which RDK controller tracks the targets RCS sends. All of them
are non-real-time modes in which the robot's internal motion generator smoothens the discrete
targets, so RCS never has to stream at 1 kHz.

| `FlexivControlMode`    | RDK mode                    | Tunable gains                             |
| ---------------------- | --------------------------- | ----------------------------------------- |
| `JOINT_POSITION`       | `NRT_JOINT_POSITION`        | none, stiff position tracking             |
| `JOINT_IMPEDANCE`      | `NRT_JOINT_IMPEDANCE`       | `joint_stiffness`, `joint_damping_ratio`  |
| `CARTESIAN_IMPEDANCE`  | `NRT_CARTESIAN_MOTION_FORCE`| `cartesian_stiffness`, `cartesian_damping_ratio`, `max_contact_wrench`, `null_space_posture` |

In the joint modes, RCS's Cartesian control modes go through the Pinocchio IK of the RCS robot
model, exactly like in simulation. In `CARTESIAN_IMPEDANCE` the target pose is handed to the
robot's own Cartesian controller instead; RCS converts its TCP (`tcp_offset`) to the TCP of the
active tool for that. Joint targets and `move_home` temporarily switch to joint impedance in that
mode, mode switches stop the arm.

Stiffness values are absolute: `joint_stiffness` in Nm/rad within `[0, nominal]`,
`cartesian_stiffness` in N/m and Nm/rad within `[0, nominal]`. `None` means nominal, `0` makes an
axis free-floating. Damping ratios are within `[0.3, 0.8]`, `None` means the RDK default of 0.7. The
nominal values of the connected robot are available as `Flexiv.nominal_joint_stiffness` and
`Flexiv.nominal_cartesian_stiffness`, and the gains can be changed at runtime:

```python
robot.set_joint_impedance(0.5 * robot.nominal_joint_stiffness)  # softer
robot.set_joint_impedance(None)  # back to nominal
```

## Sync and async control

The RDK setters are non-blocking, they return once the target was delivered and the robot re-plans
its trajectory online. `FlexivConfig.async_control` and `FlexivGripperConfig.async_control` select
what the RCS setters do on top of that:

- `async_control=True`: `set_joint_position`, `set_cartesian_position` and the gripper setters return
  right away. Use this to stream targets, e.g. for teleoperation or policies running at a fixed rate.
- `async_control=False`: they poll the measured state every 5 ms until the target is within
  `joint_tolerance` or `cartesian_tolerance` (`width_tolerance` for the gripper) and return on
  `command_timeout` at the latest. In the impedance modes a target behind an obstacle is never
  reached, which is what the timeout is for. The gripper additionally returns once the fingers
  stopped on an object.

`move_home` always blocks and uses the reduced `move_home_velocity`. Velocity and acceleration
limits of the internal trajectory generator are configured through `max_joint_velocity`,
`max_joint_acceleration` and the Cartesian counterparts.

## Usage

```python
from rcs.envs.base import ControlMode
from rcs_flexiv.configs import DefaultRizon4SHardwareEnv

env_creator = DefaultRizon4SHardwareEnv()
env_creator.robot_sn = "Rizon4s-063650"
env_creator.tool_name = "Flexiv-GN01"

cfg = env_creator.config()
cfg.control_mode = ControlMode.CARTESIAN_TQuat
cfg.robot_cfg.async_control = False

env = env_creator.create_env(cfg)
obs, info = env.reset()
```

Without the env wrappers:

```python
import rcs
from rcs import common
from rcs_flexiv.hw import Flexiv, FlexivConfig, FlexivControlMode, FlexivGripper, FlexivGripperConfig

cfg = FlexivConfig(robot_sn="Rizon4s-063650", control_mode=FlexivControlMode.JOINT_IMPEDANCE, dof=7, ...)
ik = common.Pin(cfg.kinematic_model_path, cfg.attachment_site)
robot = Flexiv(cfg, ik)
gripper = FlexivGripper(FlexivGripperConfig(device_name="Flexiv-GN01"), robot)

robot.move_home()
gripper.grasp()
robot.close()
```

The gripper is a device of the same robot connection, so `FlexivGripper` is created from the
`Flexiv` instance and `create_env` does the same. `Flexiv.get_state()` returns the external TCP
wrench and joint torque estimates of the RDK next to the fault and mode flags.

See `src/rcs_flexiv/scripts/test_robot.py` for a bring-up script covering both control modes, a
stiffness change and the gripper, and
[examples/rizon4s/rizon4s_env_cartesian_control.py](../../examples/rizon4s/rizon4s_env_cartesian_control.py)
for a maintained Cartesian control example that runs in simulation and on hardware.

## Dual arm setups

`DefaultRizon4SDualMultiHardwareEnv` combines two arms with `MultiRobotWrapper`, one env per robot
connection, for the duo arrangement of `rcs/rizon4s_duo`: bases 0.3 m apart in y, each tilted 45
degrees about x away from the other arm. Actions and observations are expressed in the shared base
frame between the two bases, `robot_to_shared_base_frame` holds the pose of each base in that frame
and can be replaced for other mounts. See
[examples/teleop/rizon.py](../../examples/teleop/rizon.py) for teleoperation of the duo with a Meta
Quest.

```python
from rcs_flexiv.configs import DefaultRizon4SDualMultiHardwareEnv

creator = DefaultRizon4SDualMultiHardwareEnv()
creator.left_sn, creator.right_sn = "Rizon4s-063650", "Rizon4s-654321"
env = creator.create_env(creator.config())
```

## Safety notes

- `Flexiv` clears minor faults and servos on during construction, and `RobotWrapper.reset` retries
  homing after `automatic_error_recovery`. Keep a hand on the E-stop when bringing the robot up.
- The default `q_home` of the Rizon 4s in RCS points the flange straight down about 0.57 m in front
  of the base. Verify it fits the setup before the first `reset`.
- Lowering the stiffness makes the arm sag under payload and follow targets less accurately.
  Changing the damping ratio away from the default can cause instabilities, as the RDK documents.
- The Grav GN-01 with 48 V supply does not initialize on power-on and ignores commands until it has
  been initialized. `FlexivGripperConfig(manual_init=True)`, the default of the env creators, triggers
  the initialization on startup, during which the fingers move to both stops.

## Simulation

The matching simulated env is registered as `rcs/rizon4s` in core RCS with the Grav GN-01 as
gripper. The kinematics of `assets/robots/rizon4s/rizon4s.xml` match Flexiv's URDF, so joint and
Cartesian targets carry over between simulation and hardware.
