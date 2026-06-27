# RCS UR5e Extension

This extension provides support for the UR5e robot in RCS.

## Installation

```shell
pip install rcs-ur5e
```

For local development:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_ur5e
```

## Usage

```python
from rcs.envs.base import ControlMode, RelativeTo
from rcs_ur5e.configs import DefaultUR5eHardwareEnv

env_creator = DefaultUR5eHardwareEnv()
env_creator.ip = "192.168.1.15"

cfg = env_creator.config()
cfg.control_mode = ControlMode.CARTESIAN_TQuat
cfg.camera_cfgs = None
cfg.max_relative_movement = 0.2
cfg.relative_to = RelativeTo.LAST_STEP

env = env_creator.create_env(cfg)
obs, info = env.reset()
```

For a maintained example, see `examples/ur5e/ur5e_env_cartesian_control.py`.
