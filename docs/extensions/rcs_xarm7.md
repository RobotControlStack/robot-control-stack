# RCS xArm7 Extension

This extension provides support for the xArm7 robot in RCS.

## Installation

```shell
pip install rcs-xarm7
```

For local development:

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

For a maintained example, see `examples/xarm7/xarm7_env_cartesian_control.py`.
