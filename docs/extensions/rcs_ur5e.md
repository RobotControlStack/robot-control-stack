# UR5e Extension

The `rcs_ur5e` extension adds UR5e hardware support and an environment creator for Robot Control Stack.

## Safety Notices

- This controller does not implement compliant force control.
- Start slowly on the teach pendant and verify the home pose before running motions.
- Treat the extension as a low-level controller, not a safety layer.

## Installation

```shell
pip install -ve extensions/rcs_ur5e
```

## Hardware Usage

For direct hardware access, create a `UR5eConfig`, then build the robot with a kinematics backend.

```python
import rcs
from rcs_ur5e.hw import UR5e, UR5eConfig

cfg = UR5eConfig(ip="192.168.25.201")
ik = rcs.common.Pin(
    cfg.kinematic_model_path,
    cfg.attachment_site,
    urdf=cfg.kinematic_model_path.endswith(".urdf"),
)
robot = UR5e(cfg, ik)
```

For a Gymnasium-style hardware environment, use `RCSUR5eEnvCreator` from `rcs_ur5e.creators`.

## Simulation Usage

The simulation examples show the current setup pattern for UR5e in MuJoCo:

- `examples/ur5e/ur5e_env_joint_control.py`
- `examples/ur5e/ur5e_env_cartesian_control.py`

Unlike the default FR3 simulation config, the UR5e examples set robot joints, actuators, attachment site, and kinematic model explicitly before creating the environment.

## Notes

- Hardware support is implemented in `extensions/rcs_ur5e/src/rcs_ur5e/hw.py`.
- The environment creator lives in `extensions/rcs_ur5e/src/rcs_ur5e/creators.py`.
- Use the example scripts as the source of truth if this page ever drifts.
