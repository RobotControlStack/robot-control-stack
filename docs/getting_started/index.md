# Getting Started

## Installation

We build and test RCS on the latest Debian and on the latest Ubuntu LTS.

### Prerequisites

1.  Install the system dependencies:

    ```shell
    sudo apt install $(cat debian_deps.txt)
    ```

2.  Create and activate Python virtual environment or conda environment:

    ```shell
    conda create -n rcs python=3.11
    conda activate rcs
    ```

3.  Install the package dependencies:

    ```shell
    pip install -r requirements.txt
    ```

### Building RCS

Build and install RCS in editable mode:

```shell
pip install -ve . --no-build-isolation
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
simulation = sim.Sim(rcs.scenes["fr3_empty_world"].mjb)
urdf_path = rcs.scenes["fr3_empty_world"].urdf
ik = rcs.common.RL(str(urdf_path))

# Configure robot
cfg = sim.SimRobotConfig()
cfg.add_id("0")
cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
robot = rcs.sim.SimRobot(simulation, ik, cfg)

# Configure gripper
gripper_cfg_sim = sim.SimGripperConfig()
gripper_cfg_sim.add_id("0")
gripper = sim.SimGripper(simulation, gripper_cfg_sim)

# Configure cameras
camera_set = SimCameraSet(simulation, {})

# Open GUI
simulation.open_gui()
sleep(5)

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
- `fr3_direct_control.py`: Direct robot control with RCS's python bindings.
- `fr3_env_joint_control.py`: Gymnasium interface with joint control.
- `fr3_env_cartesian_control.py`: Gymnasium interface with Cartesian control.

Most examples work both in the MuJoCo simulation as well as on hardware (with appropriate extensions installed).
