# RCS YAM Extension

This extension provides support for the I2RT YAM arm in RCS, built on the
[i2rt](https://github.com/i2rt-robotics/i2rt) Python driver.

## Installation

`i2rt` is not published on PyPI and is pinned as a direct git reference, so this extension is
installed from a checkout:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_yam
```

Bring up the CAN interface of the arm before use:

```shell
sudo ip link set can0 up type can bitrate 1000000
```

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

## Sync and async control

The i2rt driver runs its PD control loop in a background thread. With
`YamConfig.async_control=True` the RCS setters return as soon as the target is handed over, with
`async_control=False` they poll the measured state until the target is reached or the timeout hits.
`move_home` interpolates the motion and always blocks.

## Notes

- Arm and gripper share one motor chain, since i2rt exposes the gripper motor as the last entry of
  the arm's chain. The gripper is created from the same robot instance.
- The `linear_4310` gripper calibrates on startup and drives the fingers to both end stops unless
  `gripper_limits_override` is set.
- There is no compliant force control and no trajectory generation in the driver. Targets further
  than `max_joint_step` away are ramped rather than sent as a step.
- The simulated counterpart is registered as `rcs/yam` in core RCS and shares the kinematics, joint
  limits and gripper stroke of the hardware.

See `extensions/rcs_yam/README.md` for the full extension documentation and
`extensions/rcs_yam/src/rcs_yam/scripts/test_robot.py` for a bring-up script. For a maintained
example, see `examples/yam/yam_env_cartesian_control.py`, which moves the TCP forward and backward in
synchronous Cartesian mode in simulation or on hardware.
