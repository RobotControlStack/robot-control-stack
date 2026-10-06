# RCS DIGIT Extension

This extension provides support for [DIGIT](https://digit.ml) tactile sensors in RCS.

The sensor is exposed as an RCS `HardwareCamera`, so it is configured and polled like any other
camera in a camera set. The robot extensions register a `digit` camera type that resolves once
this extension is installed.

## Installation

```shell
pip install rcs-digit
```

For local development:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_digit
```
