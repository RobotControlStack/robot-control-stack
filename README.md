<div align="center">
  <img src="https://raw.githubusercontent.com/RobotControlStack/robotcontrolstack.github.io/refs/heads/master/static/images/rcs_logo_line.svg" alt="rcs logo" width="60%">

  ### A lean, ROS-free Sim-to-Real framework for training and deploying Vision-Language-Action (VLA) models and Reinforcement Learning (RL) agents.

  [![Documentation](https://img.shields.io/badge/docs-robotcontrolstack.org-blue.svg)](https://robotcontrolstack.org)
  [![Paper](https://img.shields.io/badge/paper-ICRA_2026-green.svg)](https://robotcontrolstack.github.io/)
  [![Release](https://img.shields.io/github/v/release/RobotControlStack/robot-control-stack?color=orange)](https://github.com/RobotControlStack/robot-control-stack/releases)
  [![License](https://img.shields.io/github/license/RobotControlStack/robot-control-stack?color=blueviolet)](https://github.com/RobotControlStack/robot-control-stack/blob/main/LICENSE)
  [![CI Status](https://github.com/RobotControlStack/robot-control-stack/actions/workflows/ci.yaml/badge.svg)](https://github.com/RobotControlStack/robot-control-stack/actions)
</div>

---

**Robot Control Stack (RCS)** is a flexible, native [Gymnasium](https://gymnasium.farama.org/) wrapper-based robot control interface designed specifically for modern robot learning and Vision-Language-Action (VLA) models. 

It completely unifies **MuJoCo simulation** and real-world physical robot control into a single, seamless API. Currently, RCS natively supports four robots out-of-the-box: **Franka FR3/Panda, xArm7, UR5e, and SO101.**

![RCS Demo](https://raw.githubusercontent.com/RobotControlStack/robotcontrolstack.github.io/refs/heads/master/static/videos/grid.webp)

## 🚀 Why use Robot Control Stack?

Traditional robotics middleware (like ROS/ROS2) and complex motion planning pipelines (like MoveIt or standard `ros2_control`) are built for asynchronous, distributed systems. This often becomes a massive bottleneck when attempting to train modern, synchronous machine learning models.

**RCS is built differently:**
* **Zero ROS Overhead:** No complex message-passing, middleware, or network configuration required. Run natively in Python with a lightweight C++ backend.
* **Frictionless Sim-to-Real:** Train your Reinforcement Learning or VLA policies in our MuJoCo Gymnasium wrapper, and deploy the *exact same code* directly to physical hardware.
* **Synchronous Execution:** Optimized specifically for the highly parallelized, synchronous data collection required by modern ML workflows.
* **Ready-to-Use Apps:** Ships with pre-built applications for data collection via teleoperation and remote model inference via [vlagents](https://github.com/RobotControlStack/vlagents).

## 🧩 Wrapper-Based Architecture

RCS utilizes a highly modular, wrapper-based architecture, allowing you to easily stack capabilities (cameras, grippers, action spaces) as needed.

<img src="docs/_static/rcs_architecture_small.svg" alt="rcs architecture diagram" width="100%">

## 💻 Example: Composing your Environment

Flexibly compose your Gymnasium environment to fit your exact training needs. *For common environment compositions, factory functions such as `rcs.envs.creators.SimEnvCreator` are provided.*

```python
from time import sleep

import gymnasium as gym
import numpy as np
from rcs._core.sim import SimConfig
from rcs.camera.sim import SimCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    GripperWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotEnv,
)
from rcs.envs.sim import GripperWrapperSim, RobotSimWrapper
from rcs.envs.utils import (
    default_mujoco_cameraset_cfg,
    default_sim_gripper_cfg,
    default_sim_robot_cfg,
)

import rcs
from rcs import sim

if __name__ == "__main__":
    # default configs
    robot_cfg = default_sim_robot_cfg(scene="fr3_empty_world")
    gripper_cfg = default_sim_gripper_cfg()
    cameras = default_mujoco_cameraset_cfg()
    sim_cfg = SimConfig()
    sim_cfg.realtime = True
    sim_cfg.async_control = True
    sim_cfg.frequency = 1  # in Hz (1 sec delay)

    simulation = sim.Sim(robot_cfg.mjcf_scene_path, sim_cfg)
    ik = rcs.common.Pin(
        robot_cfg.kinematic_model_path,
        robot_cfg.attachment_site,
        urdf=False,
    )

    # base env
    robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
    env: gym.Env = RobotEnv(robot, ControlMode.CARTESIAN_TQuat)

    # gripper
    gripper = sim.SimGripper(simulation, gripper_cfg)
    env = GripperWrapper(env, gripper, binary=True)

    env = RobotSimWrapper(env, simulation)
    env = GripperWrapperSim(env, gripper)

    # camera
    camera_set = SimCameraSet(simulation, cameras, physical_units=True, render_on_demand=True)
    env = CameraSetWrapper(env, camera_set, include_depth=True)

    # relative actions bounded by 10cm translation and 10 degree rotation
    env = RelativeActionSpace(env, max_mov=(0.1, np.deg2rad(10)), relative_to=RelativeTo.LAST_STEP)

    env.get_wrapper_attr("sim").open_gui()
    # wait for gui to open
    sleep(1)
    env.reset()

    # access low level robot api to get current cartesian position
    print(env.unwrapped.robot.get_cartesian_position())

    for _ in range(10):
        # move 1cm in x direction (forward) and close gripper
        act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": [0]}
        obs, reward, terminated, truncated, info = env.step(act)
        print(obs)

```

> **Note:** This and other examples can be found in the [`examples/`]() folder.

## 🛠️ Installation

### From Source

Make sure that common build tools (i.e., `build-essential`) and a C++ compiler like `gcc` or `clang` are installed on your system/conda/docker.

*RCS works best in Python 3.11, and all extensions have been tested to work in 3.11.*

* *For Python >3.11: The `rcs_realsense` extension won't work due to the `pyrealsense2` version RCS utilizes.*
* *For Python >3.12: The `ompl` python module is currently not available on PyPI. If OMPL is not used, it is safe to remove this dependency in `pyproject.toml`.*

```shell
# setup environment
conda create -n rcs python=3.11
conda activate rcs
conda install conda-forge::glfw
# or sudo apt install $(cat debian_deps.txt)
pip install 'pip>=25.1'
pip install --group build_deps

# install rcs
pip install -ve .

```

### Via PyPI/pip

*Coming soon...*

## 🦾 Hardware Extensions

RCS supports various hardware extensions to seamlessly connect your policies to the real world (e.g., FR3, xArm7, RealSense). These are located in the `extensions` directory.

To install a specific robot extension (example for Franka FR3):

```shell
sudo apt install $(cat extensions/rcs_fr3/debian_deps.txt)
pip install -ve extensions/rcs_fr3
```

For a full list of extensions and detailed documentation, visit **[robotcontrolstack.org/extensions](https://robotcontrolstack.org/extensions)**.

## ⚠️ Troubleshooting & FAQ
* **License error or group argument not found during installation?** Make sure you are using a pip version `>=25.1` and setuptools version `>=45`.
* **Dependency error during installation?** Make sure you are using Python 3.11. RCS extensions currently do not support 3.12+ due to OMPL and RealSense dependencies.
* **Simulation is running too slow?** Check that you have enable on-demand rendering: `SimCameraSet(..., render_on_demand=True)` to render camera frames only once per step. Resolution and number of cameras in the scene has a large impact on simulation speed. Make sure to use a decent GPU when rendering is enabled.


## 📚 Documentation

For full documentation, including advanced installation, modular usage, and API references, please visit:
👉 **[robotcontrolstack.org](https://robotcontrolstack.org)**

## 🤝 Contribution

We welcome contributions from the robotics and ML community! For contribution guidelines, please check out **[robotcontrolstack.org/contributing](https://robotcontrolstack.org/contributing)**.

## 📝 Citation

If you find RCS useful for your academic work please consider citing it:

```bibtex
@inproceedings{juelg2026robotcontrolstack,
  title={{Robot Control Stack}: {A} Lean Ecosystem for Robot Learning at Scale}, 
  author={Tobias J{\"u}lg and Pierre Krack and Seongjin Bien and Yannik Blei and Khaled Gamal and Ken Nakahara and Johannes Hechtl and Roberto Calandra and Wolfram Burgard and Florian Walter},
  year={2026},
  booktitle={Proc.~of the IEEE Int.~Conf.~on Robotics \& Automation (ICRA)},
  note={Accepted for publication.}
}
```

For more scientific information and supplementary videos, visit the **[paper website](https://robotcontrolstack.github.io/)**.

