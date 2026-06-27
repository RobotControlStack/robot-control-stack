# RCS Tacto Extension

Integration with the Tacto tactile sensor simulator.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_tacto>

## Installation

Install from PyPI:

```shell
pip install rcs-tacto
```

Warning: plain `pip install rcs-tacto` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_tacto
```

An example environment is available in [examples/fr3/grasp_digit_demo.py](../../examples/fr3/grasp_digit_demo.py).

In particular, `FR3TactoSimplePickUpSimEnvCreator` shows how Tacto is integrated into the RCS stack.
