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

## Serials

```shell
python -m rcs_robotiq2f85 serials
# the following command also works
udevadm info -a -n /dev/ttyUSB0 | grep serial
```

Lists every `/dev/ttyUSB*` device with its serial number. The serial number is what you pass as
`serial_number` to `RobotiQ2F85GripperConfig`.

To access the serial port, add yourself to the `dialout` group, then log out and back in:

```shell
sudo adduser $USER dialout
```

As a temporary alternative that is reset when the gripper is replugged:

```shell
sudo chmod 666 /dev/ttyUSB0
```

## Usage

```python
from rcs_robotiq2f85 import RobotiQ2F85GripperConfig, RobotiQGripper

gripper = RobotiQGripper(RobotiQ2F85GripperConfig(serial_number="<YOUR_SERIAL_NUMBER>"))
gripper.reset()
gripper.shut()
print(gripper.get_normalized_width())
```
