# RCS Panda Extension

This extension provides support for the Franka Emika Panda robot in RCS.

## Installation

```shell
sudo apt install $(cat extensions/rcs_panda/debian_deps.txt)
pip install rcs-panda
```

For local development from this repository:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_panda --no-build-isolation
```

## Usage

```python
from rcs.envs.base import ControlMode, RelativeTo
from rcs_panda.configs import DefaultPandaHardwareEnv

env_creator = DefaultPandaHardwareEnv()
env_creator.ip = "192.168.4.100"

cfg = env_creator.config()
cfg.control_mode = ControlMode.CARTESIAN_TQuat
cfg.camera_cfgs = None
cfg.max_relative_movement = 0.2
cfg.relative_to = RelativeTo.LAST_STEP

env = env_creator.create_env(cfg)
obs, info = env.reset()
print(env.get_wrapper_attr("robot").get_cartesian_position())
```

For a maintained example, see `extensions/rcs_panda/src/rcs_panda/panda_env_cartesian_control.py`.
