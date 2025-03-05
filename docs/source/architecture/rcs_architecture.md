# Robot Control Stack architecture
<p align="center">
  <img src="../_static/images/rcs_overview_of_layered_architecture.png" />
</p>

A core contribution of RCS is a simple yet versatile
architecture that allows for easy extension and customization.
The architecture is organized in multiple layers that represent
increasing levels of abstraction from platform-specific implementation details.
While lower layers are implemented in
C++, higher layers are implemented in Python or provide
bindings for Python. Rapid prototyping and performance-
critical code are therefore supported equally well.
RCS uses [MuJoCo](https://ieeexplore.ieee.org/document/6386109) as its simulator and the [Robotics
Library](https://ieeexplore.ieee.org/document/8202232) for inverse kinematics. There is support for
Franka robots through the native library provided by the
manufacturer. New robot types can be added by implementing a small set of API functions. 
Both the simulation and the actual hardware share a unified interface for a
seamless transition between the two. Adding sensors and
actuators to the simulation is managed through a frequency-
based callback mechanism that interfaces with the global
simulation loop.

At the most abstract level, RCS implements an easy-to-use
[Gymnasium-style API](https://arxiv.org/abs/2407.17032). This interface is not only widely
used in reinforcement learning but also enables the flexible
composition of environment wrappers for adding features.

For example, the collision checker depicted in the above figure is
implemented as a wrapper that checks motion commands
in simulation and filters them to prevent collisions.