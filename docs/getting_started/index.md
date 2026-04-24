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
robot_meta = rcs.ROBOTS[rcs.common.RobotType.FR3]
simulation = sim.Sim(rcs.SCENE_PATHS["empty_world"])
ik = rcs.common.Pin(robot_meta.mjcf_model_path, robot_meta.attachment_site)

# Configure robot
cfg = sim.SimRobotConfig()
cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
robot = rcs.sim.SimRobot(simulation, ik, cfg)

# Configure gripper
gripper_cfg_sim = sim.SimGripperConfig()
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
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3
import numpy as np

scene = EmptyWorldFR3()
cfg = scene.config()
cfg.control_mode = ControlMode.JOINTS
cfg.max_relative_movement = np.deg2rad(5)
cfg.relative_to = RelativeTo.LAST_STEP
env_rel = scene.create_env(cfg)

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
