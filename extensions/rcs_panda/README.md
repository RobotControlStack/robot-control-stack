# RCS Panda Extension

Support for the Franka Emika Panda robot in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_panda>

## Installation

Install the Debian dependency first:

```shell
sudo apt install $(cat debian_deps.txt)
```

Install from PyPI:

```shell
pip install rcs-panda
```

Warning: plain `pip install rcs-panda` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout for development:

```shell
pip install -ve . --no-build-isolation
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_panda --no-build-isolation
```

Add your Panda Desk credentials to a `.env` file:

```env
DESK_USERNAME=...
DESK_PASSWORD=...
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

For a maintained example, see [src/rcs_panda/panda_env_cartesian_control.py](src/rcs_panda/panda_env_cartesian_control.py).

## CLI

```shell
python -m rcs_panda --help
```
