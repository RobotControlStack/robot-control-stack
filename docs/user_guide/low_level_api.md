# Low-Level API

At its core, RCS provides a C++ interface that defines all functions needed to control a robot in an abstract manner. This interface has Python bindings, allowing for direct control without the overhead of the Gymnasium interface.

## C++ Interface

The C++ layer handles:
- **Real-time Control**: Communication with robot hardware drivers.
- **Simulation Stepping**: Interfacing with MuJoCo.
- **Kinematics**: Fast IK/FK calculations using Pinocchio.

## Python Bindings

The Python bindings expose the C++ functionality to Python. This allows you to:
- Create `SimRobot` or `HardwareRobot` instances.
- Send joint or Cartesian commands directly.
- Read robot state (positions, velocities, torques).
- Interface with sensors (cameras, grippers).

### Example: Direct Control

```python
import rcs.sim as sim
# ... setup ...
robot.set_cartesian_position(target_pose)
simulation.step_until_convergence()
```

## Adding New Robots

Support for new robots can be implemented in both C++ and Python.
- **C++**: Implement the `Robot` interface for high-performance drivers.
- **Python**: Implement the python-side interface for easier prototyping or python-only drivers.

The base environment is implementation-agnostic and works with any robot that adheres to the interface.
