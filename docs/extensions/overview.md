# Extensions Overview

RCS is designed to be modular. Core functionality is kept minimal, while specific hardware support and additional features are provided through **extensions**.

## What is an Extension?

An extension is a separate Python package that integrates with RCS. Extensions can provide:
- **Hardware Support**: Drivers for specific robots (e.g., FR3, xArm7) or sensors (e.g., RealSense).
- **Simulation Assets**: MJCF/URDF files for new robots.
- **Additional Functionality**: Integrations with other libraries (e.g., Robotics Library).

## Installing Extensions

Extensions are typically installed via `pip`.

```shell
pip install -ve extensions/rcs_fr3
```

## Available Extensions

RCS comes with several supported extensions:

- **rcs_fr3**: Support for the Franka Research 3 robot.
- **rcs_panda**: Support for the Franka Emika Panda robot.
- **rcs_xarm7**: Support for the xArm7 robot.
- **rcs_ur5e**: Support for the UR5e robot.
- **rcs_so101**: Support for the SO101 robot.
- **rcs_realsense**: Support for Intel RealSense cameras.
- **rcs_usb_cam**: Support for generic USB webcams.
- **rcs_tacto**: Integration with the Tacto tactile sensor simulator.
- **rcs_robotics_library**: Integration with the Robotics Library (RL).

## Creating Extensions

You can create your own extensions to add support for new hardware or features.
- [Creating a Python Extension](../development/python_extension.md)
- [Creating a C++ Extension](../development/cpp_extension.md)
