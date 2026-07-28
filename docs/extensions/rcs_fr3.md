# RCS FR3 Extension

This extension provides support for the Franka Research 3 (FR3) robot in RCS.

## Installation

```shell
sudo apt install $(cat extensions/rcs_fr3/debian_deps.txt)
pip install rcs-fr3
```

For local development from this repository:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_fr3 --no-build-isolation
```

### Configuration

Add your FR3 credentials to a `.env` file:

```bash
DESK_USERNAME=...
DESK_PASSWORD=...
```

## Usage

```python
from rcs.envs.base import ControlMode, RelativeTo
from rcs_fr3.configs import DefaultFR3HardwareEnv

env_creator = DefaultFR3HardwareEnv()
env_creator.ip = "192.168.101.1"

cfg = env_creator.config()
cfg.control_mode = ControlMode.CARTESIAN_TQuat
cfg.camera_cfgs = None
cfg.max_relative_movement = 0.5
cfg.relative_to = RelativeTo.LAST_STEP

env = env_creator.create_env(cfg)
obs, info = env.reset()
print(env.get_wrapper_attr("robot")["right"].get_cartesian_position())
```

If you need direct access to single-robot config fields like `robot_cfg` or `gripper_cfg`,
use `DefaultSingleFR3HardwareEnv` instead.

For a maintained example, see `examples/fr3/fr3_env_cartesian_control.py`.

## CLI

The extension defines useful commands to handle the FR3 robot without the need to use the Desk Website.

```shell
python -m rcs_fr3 --help
```
