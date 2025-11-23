# Gymnasium Interface

The high-level interface of RCS is based on [Gymnasium](https://gymnasium.farama.org/). This allows for easy integration with Reinforcement Learning libraries and standard control pipelines.

## Environment Creation

Environments are created using the `SimEnvCreator` (for simulation) or `HardwareEnvCreator` (for real robots).

```python
from rcs.envs.creators import SimEnvCreator
from rcs.envs.base import ControlMode, RelativeTo

env = SimEnvCreator()(
    control_mode=ControlMode.JOINTS,
    # ... configuration objects ...
)
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
