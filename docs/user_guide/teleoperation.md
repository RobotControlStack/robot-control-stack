# Teleoperation

RCS supports teleoperation using various devices, including VR headsets (Meta Quest 3) and specialized hardware like GELLO.

## VR Teleoperation (Meta Quest 3)

The [IRIS platform](https://intuitive-robots.github.io/iris-project-page/index.html) is used to stream controller poses from the Meta Quest 3 to RCS.

### Setup

1.  **Install IRIS**: Follow the instructions on the [IRIS Meta Quest repository](https://github.com/intuitive-robots/IRIS-Meta-Quest3).
2.  **Install SimPub**: The IRIS Python client.
    ```shell
    pip install -r examples/teleop/requirements.txt
    ```
3.  **Calibrate**: Run the alignment script to map the VR coordinate system to the robot's base frame.
    ```shell
    python examples/teleop/quest_align_frame.py
    ```

### Running Teleoperation

The `franka.py` example demonstrates teleoperating a Franka FR3 (sim or hardware). It uses the `RelativeActionSpace` wrapper to apply delta movements from the VR controllers.

```python
# From examples/teleop/franka.py
# The script configures the environment to follow controller poses
# and records data using the StorageWrapper.
```

## Hardware Teleoperation (GELLO)

GELLO is a 3D-printed, low-cost teleoperation device. RCS supports GELLO for various robots.

### Setup

1.  Connect the GELLO device via USB.
2.  Install dependencies:
    ```shell
    pip install -r examples/teleop/requirements.txt
    ```
3.  Configure the `GelloConfig` in your teleoperation script with the appropriate USB IDs.

## Key Concepts

- **Relative Control**: Teleoperation typically uses delta-movements. The `RelativeActionSpace` wrapper is used to translate absolute device poses into relative robot movements.
- **Visual Feedback**: When teleoperating in simulation, the GUI provides real-time visual feedback. For hardware, cameras (e.g., RealSense) can be used to provide a first-person view.
- **Data Collection**: Teleoperation is often used to collect expert demonstrations for Imitation Learning. This is integrated via the `StorageWrapper`.
