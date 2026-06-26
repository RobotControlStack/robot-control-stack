# RCS FR3 Extension

Support for the Franka Research 3 (FR3) robot in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_fr3>

## Installation

Install the Debian dependency first:

```shell
sudo apt install $(cat debian_deps.txt)
```

Install from PyPI:

```shell
pip install rcs-fr3
```

Warning: plain `pip install rcs-fr3` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout for development:

```shell
pip install -ve . --no-build-isolation
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_fr3 --no-build-isolation
```

For `libfranka` version details, see <https://robotcontrolstack.org/extensions/libfranka_versions>.

Add your FR3 Desk credentials to a `.env` file:

```env
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
print(env.get_wrapper_attr("robot").get_cartesian_position())
```

For a maintained end-to-end example, see [examples/fr3/fr3_env_cartesian_control.py](../../examples/fr3/fr3_env_cartesian_control.py).

## CLI

```shell
python -m rcs_fr3 --help
```
