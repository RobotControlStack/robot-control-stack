# RCS ZED Extension

This extension provides support for Stereolabs ZED cameras in RCS.

## Installation

- You need a PC with an Nvidia GPU and CUDA 12.8 installed.
- Install the ZED SDK from Stereolabs.
- Install the Python bindings from `zed-python-api`.

```shell
pip install rcs-zed
```

For local development:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_zed
```

## CLI

```shell
python -m rcs_zed serials
python -m rcs_zed rgb-view
```
