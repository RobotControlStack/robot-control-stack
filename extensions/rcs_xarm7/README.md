# RCS xArm7 Extension

Support for the xArm7 robot in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_xarm7>

## Installation

Install from PyPI:

```shell
pip install rcs-xarm7
```

Warning: plain `pip install rcs-xarm7` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_xarm7
```

## Usage

```python
from rcs.envs.base import ControlMode, RelativeTo
from rcs_xarm7.configs import DefaultXArm7HardwareEnv

env_creator = DefaultXArm7HardwareEnv()
env_creator.ip = "192.168.1.245"

cfg = env_creator.config()
cfg.control_mode = ControlMode.CARTESIAN_TQuat
cfg.max_relative_movement = 0.5
cfg.relative_to = RelativeTo.LAST_STEP

env = env_creator.create_env(cfg)
obs, info = env.reset()
```

For a maintained example, see [examples/xarm7/xarm7_env_cartesian_control.py](../../examples/xarm7/xarm7_env_cartesian_control.py).
