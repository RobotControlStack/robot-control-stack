# Getting Started

## Installation

We build and test RCS on the latest Debian and on the latest Ubuntu LTS.

### Prerequisites

1.  Install the system dependencies:

    ```shell
    sudo apt install $(cat debian_deps.txt)
    ```

2.  Create and activate Python virtual environment or conda environment:

    RCS requires **Python >= 3.10**. We strongly recommend **Python 3.11** for full compatibility with all extensions (e.g., `rcs_realsense`).

    ```{note}
    - **Python 3.11** is preferred for development.
    - **Python > 3.11**: `rcs_realsense` may have compatibility issues due to `pyrealsense2` limitations.
    - **Python > 3.12**: The `ompl` dependency is currently unavailable on PyPI.
    ```

    ```shell
    conda create -n rcs python=3.11
    conda activate rcs
    pip install 'pip>=25.1'
    ```

3.  Install the package dependencies:

    ```shell
    pip install --group build_deps
    ```

### Building RCS

Build and install RCS in editable mode:

```shell
pip install -ve .
```

For a docker deployment, see the `docker` folder in the repository.

## Basic Usage

The python package is called `rcs`.

### Direct Robot Control

Here is a simple example of direct robot control using the low-level API:

```python
import rcs
from rcs import sim
from rcs._core.sim import CameraType
from rcs.camera.sim import SimCameraConfig, SimCameraSet
from time import sleep
import numpy as np

# Load simulation scene
scene = rcs.scenes["fr3_empty_world"]
simulation = sim.Sim(scene.mjb or scene.mjcf_scene)

# Configure robot
cfg = sim.SimRobotConfig()
cfg.add_postfix("_0")
cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())

# Configure IK solver (using Pinocchio)
ik = rcs.common.Pin(
    cfg.kinematic_model_path,
    cfg.attachment_site,
    urdf=cfg.kinematic_model_path.endswith(".urdf"),
)
robot = rcs.sim.SimRobot(simulation, ik, cfg)

# Configure gripper
gripper_cfg_sim = sim.SimGripperConfig()
gripper_cfg_sim.add_postfix("_0")
gripper = sim.SimGripper(simulation, gripper_cfg_sim)

# Configure cameras
camera_set = SimCameraSet(simulation, {})

# Open GUI
simulation.open_gui()
sleep(2)

# Step the robot 10 cm in x direction
robot.set_cartesian_position(
    robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0.1, 0, 0]))
)

# Close gripper
gripper.grasp()

# Step simulation
simulation.step_until_convergence()
input("press enter to close")
```

### Gymnasium Interface

RCS provides a high-level [Gymnasium](https://gymnasium.farama.org/) interface for Reinforcement Learning and general control.

```python
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)
from rcs.envs.base import ControlMode, RelativeTo
import numpy as np

# Create environment
env_rel = SimEnvCreator()(
    control_mode=ControlMode.JOINTS,
    robot_cfg=default_sim_robot_cfg(),
    gripper_cfg=default_sim_gripper_cfg(),
    cameras=default_mujoco_cameraset_cfg(),
    max_relative_movement=np.deg2rad(5),
    relative_to=RelativeTo.LAST_STEP,
)

# Open GUI
env_rel.get_wrapper_attr("sim").open_gui()

# Run loop
for _ in range(100):
    obs, info = env_rel.reset()
    for _ in range(10):
        # Sample random relative action and execute it
        act = env_rel.action_space.sample()
        print(act)
        obs, reward, terminated, truncated, info = env_rel.step(act)
        print(obs)
```

## Examples

Check out the python examples in the `examples` folder of the repository.
- `examples/fr3/fr3_direct_control.py`: Direct robot control with RCS's python bindings.
- `examples/fr3/fr3_env_joint_control.py`: Gymnasium interface with joint control.
- `examples/fr3/fr3_env_cartesian_control.py`: Gymnasium interface with Cartesian control.

Most examples work both in the MuJoCo simulation as well as on hardware (with appropriate extensions installed).
