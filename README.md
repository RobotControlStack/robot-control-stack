# Robot Control Stack
RCS is a unified and multilayered robot control interface over a MuJoCo simulation and real world robot currently implemented for the FR3.
## Requirements
We build and test RCS on the latest Debian and on the latest Ubuntu LTS.

## Installation
1. Install the system dependencies:
```shell
sudo apt install $(cat debian_deps.txt)
```
2. Create, activate and configure a [Python virtual environment](https://docs.python.org/3/library/venv.html):
```shell
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements_dev.txt
pip config --site set global.no-build-isolation false
```
3. Build and install RCS:
```shell
pip install -ve .
```

## Docker (GUI + GPU + HW add-ons)

**Prereqs:** Docker + docker-compose, X11 on host, NVIDIA driver + NVIDIA Container Toolkit (legacy `runtime: nvidia`).  
**Layout:** `Docker/Dockerfile`, overrides in `compose/` (`base.yml`, `gui.yml`, `gpu.yml`, `hw.yml`).

### Build the image
`docker-compose -f compose/base.yml build dev`

### (GUI) allow X access (host)
`export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}`  
`xhost +si:localuser:root`

### Run container with GUI + GPU + HW and open a shell
`docker-compose -f compose/base.yml -f compose/gui.yml -f compose/gpu.yml -f compose/hw.yml run --rm run bash`  
*(Use fewer `-f` files for lighter setups, e.g., GUI+GPU without HW.)*

### Inside the container
`pip install -v -e extensions/rcs_fr3`  
`cd examples`  
`python fr3_env_cartesian_control.py`

### Troubleshooting
- **`nvidia-smi` missing in container:** ensure it exists on host at `/usr/bin/nvidia-smi` (GPU override bind-mounts it).  
- **GUI can’t open:** re-run the `xhost` command and confirm `$DISPLAY` is set on the host.  


## Usage
The python package is called `rcs`.

### Direct Robot Control
Simple direct robot control:
```python
import rcs
from rcs import sim
from rcs._core.sim import CameraType
from rcs.camera.sim import SimCameraConfig, SimCameraSet
simulation = sim.Sim(rcs.scenes["fr3_empty_world"]["mjb"])
urdf_path = rcs.scenes["fr3_empty_world"]["urdf"]
ik = rcs.common.RL(str(urdf_path))
cfg = sim.SimRobotConfig()
cfg.add_id("0")
cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
robot = rcs.sim.SimRobot(simulation, ik, cfg)

gripper_cfg_sim = sim.SimGripperConfig()
gripper_cfg_sim.add_id("0")
gripper = sim.SimGripper(simulation, gripper_cfg_sim)

# add camera to have a rendering gui
cameras = {
    "wrist": SimCameraConfig(
        identifier="wrist_0",
        type=CameraType.fixed,
        resolution_width=640,
        resolution_height=480,
        frame_rate=30,
    ),
}
camera_set = SimCameraSet(simulation, cameras)
simulation.open_gui()
robot.set_cartesian_position(
    robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0.05, 0, 0]))
)
gripper.grasp()
simulation.step_until_convergence()
```
### Gym Env Interface
```python
from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)
from rcs.envs.base import ControlMode, RelativeTo
env_rel = SimEnvCreator()(
    control_mode=ControlMode.JOINTS,
    collision_guard=False,
    robot_cfg=default_sim_robot_cfg(),
    gripper_cfg=default_sim_gripper_cfg(),
    cameras=default_mujoco_cameraset_cfg(),
    max_relative_movement=np.deg2rad(5),
    relative_to=RelativeTo.LAST_STEP,
)
env_rel.get_wrapper_attr("sim").open_gui()

for _ in range(10):
    obs, info = env_rel.reset()
    for _ in range(10):
        # sample random relative action and execute it
        act = env_rel.action_space.sample()
        print(act)
        obs, reward, terminated, truncated, info = env_rel.step(act)
        print(obs)
        if truncated or terminated:
            logger.info("Truncated or terminated!")
            return
```
### Examples
Checkout the python examples in the [examples](examples) folder:
- [fr3_direct_control.py](examples/fr3.py) shows direct robot control with RCS's python bindings
- [fr3_env_joint_control.py](examples/env_joint_control.py) and [fr3_env_cartesian_control.py](examples/env_cartesian_control.py) demonstrates RCS's high level [gymnasium](https://gymnasium.farama.org/) interface both for joint- and end effector space control
All of these examples work both in the MuJoCo simulation as well as on your hardware FR3.


### Hardware Extensions
To enable hardware usage in RCS, install the needed hardware extensions via pip. RCS itself comes with a couple of supported extensions e.g. control of the FR3 via the [`rcs_fr3`](extensions/rcs_fr3) extension. All native supported extension are located in [extensions](extensions).
To install extensions:
```shell
pip install -ve extensions/rcs_fr3
```
For more details real the readme file of the respective extension.

After the required hardware extensions are installed the examples also above work on real hardware:
Switch to hardware by setting the following flag:
```python
ROBOT_INSTANCE = RobotPlatform.SIMULATION
# ROBOT_INSTANCE = RobotPlatform.HARDWARE
```

#### Command Line Interface
Some modules include command line interfaces, e.g. rcs_fr3 defines useful commands to handle the FR3 robot without the need to use the Desk Website.
You can see the available subcommands as follows:
```shell
python -m rcs_fr3 --help
python -m rcs_realsense --help
```

## Development
### Formatting and Linting
```shell
# check for c++ formatting errors
make cppcheckformat
# fix them
make cppformat
# Linting with clang tidy
make cpplint
# check for python formatting errors
make pycheckformat
# fix them
make pyformat
# Linting with ruff and mypy
make pylint
# Testing
make pytest
```
### Stub Files for Python Bindings
We use autogenerated python stub files (`.pyi`) in the [`_core`](python/rcs/_core/) folder to show our linters the expected types of the C++ Python bindings.
If the python bindings in the C++ code have changed you might need to regenerate them by using:
```shell
make stubgen
```

### Develop Your Own Hardware Extension
TODO

