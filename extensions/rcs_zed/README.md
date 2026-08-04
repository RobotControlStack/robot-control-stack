# RCS ZED Extension

Support for Stereolabs ZED cameras in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_zed>

## Installation

- You need a PC with an Nvidia GPU and CUDA 12.8 installed.
- Install the ZED SDK from [Stereolabs](https://www.stereolabs.com/en-fr/developers/release).
- Install the Python bindings from the [zed-python-api](https://github.com/stereolabs/zed-python-api).

Install from PyPI:

```shell
pip install rcs-zed
```

Warning: plain `pip install rcs-zed` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_zed
```

## Calibration

`default_zed(...)` is standalone and uses RCS's identity
`DummyCalibrationStrategy` by default. To use measured extrinsics, pass one
calibration strategy per logical camera:

```python
from rcs_zed.utils import default_zed

calibration = {
    "wrist": my_wrist_calibration,
    "scene": my_scene_calibration,
}
cameras = default_zed(
    {"wrist": "12345678", "scene": "87654321"},
    calibration_strategy=calibration,
)
```

Each value must implement the `rcs.camera.hw.CalibrationStrategy` protocol.
This keeps ZED installation independent of other camera extensions and lets
applications choose the calibration method that matches their robot setup.

## CLI

```shell
python -m rcs_zed serials
python -m rcs_zed rgb-view
```

## Permissions

```shell
sudo usermod -a -G video,plugdev $USER
```

## Connection Diagnostics

- Use `ZED_Diagnostic` first if something appears broken.
- See the Stereolabs USB troubleshooting guide: <https://support.stereolabs.com/hc/en-us/articles/207635225-How-to-fix-USB-3-0-bandwidth-and-connection-issues>
