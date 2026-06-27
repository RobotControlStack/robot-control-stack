# RCS Robotics Library Extension

Integration with the [Robotics Library (RL)](https://www.roboticslibrary.org/) for kinematics and path planning.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_robotics_library>

## Installation

Install the Debian dependencies first:

```shell
sudo apt install $(cat debian_deps.txt)
```

Install from PyPI:

```shell
pip install rcs-robotics-library
```

Warning: plain `pip install rcs-robotics-library` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout for development:

```shell
pip install -ve . --no-build-isolation
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_robotics_library --no-build-isolation
```

## Usage

```python
from rcs_robotics_library import rl
```
