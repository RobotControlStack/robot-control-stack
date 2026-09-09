# RCS Robotiq2F85 Extension

This extension provides support for Robotiq 2F-85 Gripper in RCS.

## Installation

```shell
pip install rcs-robotiq2f85
```

For local development:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_robotiq2f85
```


## Serial Numbers

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
