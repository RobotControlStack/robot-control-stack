# RCS Tilburg Hand Extension

Support for the Tilburg Hand in RCS, built on the
[`tilburg-hand`](https://pypi.org/project/tilburg-hand/) driver.

This extension depends on [`rcs-core`](https://pypi.org/project/rcs-core/).
Documentation: <https://robotcontrolstack.org/extensions/rcs_tilburg_hand>

This covers the **hardware** hand only. The simulated hand, `rcs.sim.SimTilburgHand`, is built into
`rcs-core` and needs nothing from here.

## Installation

Install from PyPI:

```shell
pip install rcs-tilburg-hand
```

Warning: plain `pip install rcs-tilburg-hand` will install the published `rcs-core` dependency from PyPI.

Install from a local checkout:

```shell
pip install -ve .
```

If you want this extension to use your local RCS checkout instead of the published `rcs-core` package, first install the main package from the repository root:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_tilburg_hand
```

## Usage

The arm extensions accept a `THConfig` wherever they take a gripper config, and attach the hand
through a `HandWrapper` once this extension is installed:

```python
from rcs_tilburg_hand.hand import THConfig

cfg.gripper_cfg = THConfig(
    calibration_file="/path/to/calibration.json",
    grasp_percentage=1,
    hand_orientation="right",
)
```
