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
