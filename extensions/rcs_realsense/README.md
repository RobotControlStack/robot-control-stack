# RCS RealSense Extension

Support for Intel RealSense cameras in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_realsense>

## Installation

Install from PyPI:

```shell
pip install rcs-realsense
```

Warning: plain `pip install rcs-realsense` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_realsense
```

## CLI

```shell
python -m rcs_realsense serials
python -m rcs_realsense rgb-view
```
