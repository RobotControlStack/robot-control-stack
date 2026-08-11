# Apps

RCS ships with ready-to-use applications for common operator workflows such as robot teleoperation and remote policy inference.

## Teleoperation

Use the Meta Quest 3 teleoperation examples when you want to collect demonstrations or directly control a robot from an operator interface. RCS provides examples for Franka and I2RT YAM arms.

- Example guide: [examples/teleop/README.md](../../examples/teleop/README.md)
- Franka example: [examples/teleop/franka.py](../../examples/teleop/franka.py)
- YAM example: [examples/teleop/yam.py](../../examples/teleop/yam.py)

The Franka example supports Meta Quest 3 and GELLO-based setups. The YAM example supports dual-arm Meta Quest 3 teleoperation in simulation or on hardware; install the [YAM extension](../extensions/rcs_yam.md) before using real YAM hardware.

## Inference

Use the Franka inference app when you want to run a remote policy server, for example through `vlagents`, against an RCS hardware environment.

- Example guide: [examples/inference/README.md](../../examples/inference/README.md)
- Main script: [examples/inference/franka.py](../../examples/inference/franka.py)
- Example config: [examples/inference/franka.json](../../examples/inference/franka.json)

The inference example expects a compatible policy server to already be running and explains the runtime keyboard commands and config fields in its README.
