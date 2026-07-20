# RCS Robotiq 2F-85 Extension

Support for the Robotiq 2F-85 gripper in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_robotiq2f85>

## Installation

Install from PyPI:

```shell
pip install rcs-robotiq2f85
```

Warning: plain `pip install rcs-robotiq2f85` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_robotiq2f85
```

Get the serial number of the gripper:

```shell
udevadm info -a -n /dev/ttyUSB0 | grep serial
```

Provide device permissions if needed:

```shell
chmod 777 /dev/ttyUSB0
```

## Usage

```python
from rcs_robotiq2f85 import RobotiQ2F85GripperConfig, RobotiQGripper

gripper = RobotiQGripper(RobotiQ2F85GripperConfig(serial_number="<YOUR_SERIAL_NUMBER>"))
gripper.reset()
gripper.shut()
print(gripper.get_normalized_width())
```

## Finger pose cache

The first call to `robotiq_2f85_finger_poses` builds the gripper's kinematics
lookup table and stores it under `$XDG_CACHE_HOME/rcs/robotiq2f85`, or under
`~/.cache/rcs/robotiq2f85` when `XDG_CACHE_HOME` is not set. Later processes
load that file instead of running the initialization simulation again.

The cache filename includes a fingerprint of the XML model, the MuJoCo version,
and the cache format. Changing any of them creates a new cache automatically.
If a closing state does not settle, its entry reuses the preceding, slightly
more-open pose and logs a warning.
