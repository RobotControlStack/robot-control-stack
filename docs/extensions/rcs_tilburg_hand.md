# RCS Tilburg Hand Extension

This extension provides support for the Tilburg Hand in RCS.

It covers the **hardware** hand only. The simulated hand, `rcs.sim.SimTilburgHand`, is implemented
in C++ inside `rcs-core` and needs nothing from this extension.

The arm extensions accept a `THConfig` wherever they take a gripper config, and attach the hand
through a `HandWrapper` once this extension is installed.

## Installation

```shell
pip install rcs-tilburg-hand
```

For local development:

```shell
pip install -ve . --no-build-isolation
pip install -ve extensions/rcs_tilburg_hand
```
