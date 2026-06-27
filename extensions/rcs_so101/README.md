# RCS SO101 Extension

Support for the SO101 robot in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_so101>

## Installation

Install from PyPI:

```shell
pip install rcs-so101
```

Warning: plain `pip install rcs-so101` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout for development:

```shell
pip install -ve . --no-build-isolation
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_so101 --no-build-isolation
```

## OpenCV Workaround

This extension depends on `lerobot`, which can pull in `opencv-python-headless` and replace the `opencv-python` variant expected by RCS.

If that happens, reinstall OpenCV like this:

```shell
pip uninstall -y opencv-python-headless opencv-python
pip install opencv-python~=4.10.0.84
```

For current examples, see [examples/so101](../../examples/so101).
