# RCS USB Camera Extension

Support for generic USB webcams in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_usb_cam>

## Installation

Install from PyPI:

```shell
pip install rcs-usb-cam
```

Warning: plain `pip install rcs-usb-cam` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_usb_cam
```

## CLI

```shell
python -m rcs_usb_cam rgb-view
```
