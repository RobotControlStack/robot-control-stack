# Architecture

RCS is designed from the ground up to support research in robot learning with large-scale generalist policies. It features a modular and easily extensible layered architecture with a unified interface for simulated and physical robots.

```{image} ../_static/rcs_architecture_small.svg
:alt: RCS Architecture
:align: center
```

## Core Design Principles

1.  **Unified Interface**: RCS provides a unified interface for both simulated (MuJoCo) and physical robots. This facilitates seamless sim-to-real transfer and enables using the simulation as a digital twin.
2.  **Layered Architecture**:
    *   **High-Level**: Applications access robots, sensors, and actuators through a [Gymnasium](https://gymnasium.farama.org/)-based Python API.
    *   **Low-Level**: The lower layers expose a C++ API for performance-critical features.
3.  **Environment Wrappers**: RCS is designed around the concept of environment wrappers. Each scene is a sequence of wrappers that can mutate the action and/or observation space.

## Environment Wrappers

An environment wrapper is a tuple $W = \langle f: S \to S', g: A' \to A, P', R' \rangle$, where $f$ and $g$ are mappings that transform the state and actions of a Markov Decision Process (MDP).

Each scene is a sequence of $n$ wrappers. At each time step, an agent issues an action $A_t$ to the wrapped MDP that is propagated through the action mutation function chain of the wrappers. The action is then passed to the base MDP (the robot interface), which produces an observation state. The observation mutation function chain updates the state and returns it to the agent.

Wrappers allow for modular additions of functionality, such as:
- **Gripper Wrapper**: Adds gripper dimensions to action/observation spaces.
- **Camera Wrapper**: Adds camera frames to the observation space.
- **Recorder Wrapper**: Records data from the scene.

## Hardware Abstraction

RCS defines interfaces and off-the-shelf wrappers for common sensors and actuators.
- **Cameras**: Wrapper implementing polling for a set of cameras.
- **End Effectors**: Wrapper for grippers or robot hands.

Adding new hardware typically involves writing a new wrapper or implementing the C++ interface for the device.

## Simulation

RCS leverages the [MuJoCo](https://mujoco.org/) physics simulation. It extends MuJoCo's API with customized functions for robotics use cases while leaving core data structures exposed.
- **Synchronous Operation**: RCS implements a callback mechanism to enable synchronous operation and interrupts (e.g., stopping on collision).
- **Digital Twin**: RCS supports running a digital twin by running both the physical robot and the MuJoCo-based simulation in parallel.

## Robotics Tool Kit

RCS integrates established tools:
- **Pinocchio**: For kinematics (IK/FK), using MuJoCo MJCF descriptions.
- **OMPL**: For motion planning.
