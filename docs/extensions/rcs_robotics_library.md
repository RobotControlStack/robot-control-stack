# RCS Robotics Library Extension

This extension provides integration with the [Robotics Library (RL)](https://www.roboticslibrary.org/) for kinematics and path planning.

## Installation

```shell
sudo apt install $(cat extensions/rcs_robotics_library/debian_deps.txt)
pip install rcs-robotics-library
```

For local development from this repository:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_robotics_library --no-build-isolation
```
