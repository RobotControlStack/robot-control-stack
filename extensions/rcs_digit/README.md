# RCS DIGIT Extension

Support for the [DIGIT](https://digit.ml) tactile sensor in RCS, built on the
[`digit-interface`](https://pypi.org/project/digit-interface/) driver.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_digit>

## Installation

Install from PyPI:

```shell
pip install rcs-digit
```

Warning: plain `pip install rcs-digit` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_digit
```

## Usage

The robot extensions register a `digit` camera type that resolves to `DigitCam` once this
extension is installed, so a DIGIT is configured like any other hardware camera:

```python
from rcs.envs.base import HardwareCameraCreatorConfig

camera_cfgs["digit"] = HardwareCameraCreatorConfig(
    camera_type_id="digit",
    camera_cfgs={"digit": BaseCameraConfig(identifier="D00001")},
)
```

`default_digit` builds those configs from a name-to-serial mapping, using the resolution and
frame rate of a named `digit-interface` stream:

```python
from rcs_digit.camera import default_digit

cameras = default_digit({"thumb": "D00001", "index": "D00002"}, stream_name="QVGA")
```
