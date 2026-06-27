# RCS UR5e Extension

Support for the UR5e robot in RCS.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_ur5e>

## Installation

Install from PyPI:

```shell
pip install rcs-ur5e
```

Warning: plain `pip install rcs-ur5e` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_ur5e
```

## Safety Notes

- This controller does not implement compliant force control.
- Verify the configured home pose before moving real hardware.
- Start at low speed and only increase after confirming motion is safe.

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

For a maintained example, see [examples/ur5e/ur5e_env_cartesian_control.py](../../examples/ur5e/ur5e_env_cartesian_control.py).
