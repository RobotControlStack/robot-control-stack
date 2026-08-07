# RCS YAM Extension

Support for the I2RT YAM arm in RCS, built on the [i2rt](https://github.com/i2rt-robotics/i2rt) Python driver.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_yam>

## Installation

`i2rt` is not published on PyPI, so it is pinned as a direct git reference in `pyproject.toml`. As a
consequence this extension is installable from a checkout but cannot be published to PyPI, and it is
not part of the wheel build workflow.

```shell
# i2rt requires ruckig==0.15.3 which has no prebuild wheel, tested to work with ruckig==0.19.4
pip install --override <(echo ruckig==0.19.4) -e extensions/rcs_yam
```

The i2rt driver talks to the motors over SocketCAN and needs kernel headers for its build
dependencies:

```shell
sudo apt install build-essential python3-dev linux-headers-$(uname -r)
```

## CAN bus setup

Each arm sits on its own CAN interface, running at 1 Mbit/s:

```shell
ls -l /sys/class/net/can*                            # list adapters
sudo ip link set can0 up type can bitrate 1000000    # bring one up
```

The i2rt repository ships `devices/install_devices.sh` to enable interfaces on boot and
`scripts/reset_all_can.sh` to reset an unresponsive adapter.

## Safety notes

- The arm is direct driven and follows a plain PD controller. There is no compliant force control
  and no trajectory generation in the driver, so avoid commanding large jumps. Targets further than
  `max_joint_step` away are ramped over `max_joint_step / max_joint_velocity` seconds instead of
  being sent as a step.
- The `linear_4310` gripper ships without stored limits and calibrates on startup, which drives the
  fingers to both end stops the moment the robot is created. Set `gripper_limits_override` on
  `YamConfig` to skip that run.
- i2rt widens the joint limits from its model by 0.15 rad for its internal clipping. RCS uses the
  tighter limits from `assets/robots/yam/yam.xml`.
- Verify the home pose before homing on real hardware, and keep the workspace clear.

## Sync and async control

The driver runs its control loop in a background thread, so commands are inherently non-blocking.
`YamConfig.async_control` selects what the RCS setters do on top of that:

- `async_control=True`: `set_joint_position` and the gripper setters return as soon as the target has
  been handed to the control loop.
- `async_control=False`: they poll the measured state every 5 ms until every joint is within
  `joint_tolerance` (or the gripper within `gripper_tolerance`) and return on `command_timeout`
  (`gripper_timeout` for gripper-only commands) at the latest. A gripper holding an object never
  reaches its commanded width, which is why it has a shorter timeout of its own.

`move_home` always blocks, since it interpolates the whole motion.

## Usage

```python
from rcs.envs.base import ControlMode
from rcs_yam.configs import DefaultYamHardwareEnv

env_creator = DefaultYamHardwareEnv()
env_creator.channel = "can0"

cfg = env_creator.config()
cfg.control_mode = ControlMode.JOINTS
cfg.robot_cfg.async_control = False

env = env_creator.create_env(cfg)
obs, info = env.reset()
```

The robot and the gripper share one motor chain, because i2rt exposes the gripper motor as the last
entry of the arm's chain. `create_env` therefore builds the `YamGripper` from the same `Yam`
instance, and closing the env closes the chain once.

Without the env wrappers:

```python
import rcs
from rcs import common
from rcs_yam.hw import Yam, YamConfig, YamGripper

cfg = YamConfig(channel="can0", async_control=False, dof=6, ...)
ik = common.Pin(cfg.kinematic_model_path, cfg.attachment_site)
robot = Yam(cfg, ik)
gripper = YamGripper(common.GripperConfig(gripper_type=common.GripperType("Yam")), robot)

robot.move_home()
gripper.grasp()
robot.close()
```

See `src/rcs_yam/scripts/test_robot.py` for a complete bring-up script covering both modes, and
[examples/yam/yam_env_cartesian_control.py](../../examples/yam/yam_env_cartesian_control.py) for a
maintained Cartesian control example that runs in simulation and on hardware.

## Bimanual setups

Each arm is a separate env on its own CAN channel, combined with `MultiRobotWrapper`:

```python
from rcs.envs.base import MultiRobotWrapper
from rcs_yam.configs import DefaultYamHardwareEnv

envs = {}
for name, channel in (("left", "can0"), ("right", "can1")):
    creator = DefaultYamHardwareEnv()
    creator.channel = channel
    envs[name] = creator.create_env(creator.config())

env = MultiRobotWrapper(envs)
```

## Simulation

The matching simulated env is registered as `rcs/yam` in core RCS. The kinematics, joint limits and
gripper stroke of `assets/robots/yam/yam.xml` match the i2rt `yam` v1 arm with the `linear_4310`
gripper, so joint and cartesian targets carry over between simulation and hardware.
