# Gymnasium Interface

The high-level interface of RCS is based on [Gymnasium](https://gymnasium.farama.org/). This allows for easy integration with Reinforcement Learning libraries and standard control pipelines.

## Environment Creation

To facilitate environment creation, RCS ships with config-based creator classes that return environments already wrapped with the most common wrappers.
Simulated environments are typically created through a scene config class such as `EmptyWorldFR3`.

```python
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3

scene = EmptyWorldFR3()
cfg = scene.config()
cfg.control_mode = ControlMode.JOINTS
cfg.relative_to = RelativeTo.LAST_STEP
env = scene.create_env(cfg)
```

Hardware environments are created using the robot-specific config creators and default config classes from the hardware extensions.
```python
from rcs.envs.base import ControlMode, RelativeTo
from rcs_fr3.configs import DefaultFR3HardwareEnv

creator = DefaultFR3HardwareEnv()
creator.ip = "192.168.100.1"
cfg = creator.config()
cfg.control_mode = ControlMode.JOINTS
cfg.camera_cfgs = None
cfg.relative_to = RelativeTo.LAST_STEP
env = creator.create_env(cfg)
```



## Control Modes

RCS supports various control modes:
- **Joint Control**: Control the robot's joint positions or velocities.
- **Cartesian Control**: Control the end-effector pose.

## Synchronous vs Asynchronous

By default, Gymnasium environments in RCS are **synchronous**. The `step()` function returns only once the action has been fully executed and the environment has reached the target state.

It is possible to configure RCS to execute actions **asynchronously**, where `step()` returns instantly. This is useful for teleoperation or high-frequency control loops where the agent doesn't wait for the robot to settle.

## Wrappers

RCS uses standard Gymnasium wrappers to extend functionality.
- **Observation Wrappers**: Modify the observation space (e.g., stacking frames, processing images).
- **Action Wrappers**: Modify the action space (e.g., normalizing actions).

## Reset Stack

RCS implements a flexible reset mechanism. When `reset()` is called, the environment can be randomized or set to a specific state based on the configuration.
