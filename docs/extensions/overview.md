# Extensions Overview

RCS is designed to be modular. Core functionality is kept minimal, while specific hardware support and additional features are provided through **extensions**.

```{note}
Extensions are supported on **Linux only**. The core `rcs-core` package (MuJoCo
simulation + Python API) also runs on macOS (Apple Silicon / arm64), but the
hardware extensions are not supported there.
```

## What is an Extension?

An extension is a separate Python package that integrates with RCS. Extensions can provide:
- **Hardware Support**: Drivers for specific robots (e.g., FR3, xArm7) or sensors (e.g., RealSense).
- **Simulation Assets**: MJCF/URDF files for new robots.
- **Additional Functionality**: Integrations with other libraries (e.g., Robotics Library).

## Installing Extensions

Extensions are typically installed from PyPI.

```shell
pip install rcs-fr3
```

For local development from a checkout of this repository, first install RCS itself from the repository root and then install the extension via its path:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_fr3 --no-build-isolation
```

## Available Extensions

RCS comes with several supported extensions:

- **rcs_fr3**: Support for the Franka Research 3 robot.
- **rcs_panda**: Support for the Franka Emika Panda robot.
- **rcs_xarm7**: Support for the xArm7 robot.
- **rcs_ur5e**: Support for the UR5e robot.
- **rcs_so101**: Support for the SO101 robot.
- **rcs_yam**: Support for the I2RT YAM arm.
- **rcs_realsense**: Support for Intel RealSense cameras.
- **rcs_usb_cam**: Support for generic USB webcams.
- **rcs_tacto**: Integration with the Tacto tactile sensor simulator.
- **rcs_robotics_library**: Integration with the Robotics Library (RL).
- **rcs_robotiq2f85**: Integration with the Robotiq 2F-85 Gripper.

## Creating Extensions

You can create your own extensions to add support for new hardware or features.
- [Creating a Python Extension](../development/python_extension.md)
- [Creating a C++ Extension](../development/cpp_extension.md)
