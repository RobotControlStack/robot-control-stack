# RCS FR3 Extension

This extension provides support for the Franka Research 3 (FR3) robot in RCS.

## Installation

```shell
# from root directory
pip install -ve extensions/rcs_fr3 --no-build-isolation
```

### Configuration

Add your FR3 credentials to a `.env` file:

```bash
DESK_USERNAME=...
DESK_PASSWORD=...
```

## Usage

```python
import rcs_fr3
from rcs_fr3._core import hw
from rcs_fr3.desk import FCI, ContextManager, Desk, load_creds_franka_desk
import rcs
import numpy as np

ROBOT_IP = "172.16.0.2" # Replace with your robot IP

user, pw = load_creds_franka_desk()
with FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False):
    urdf_path = rcs.scenes["fr3_empty_world"].urdf
    ik = rcs.common.RL(str(urdf_path))
    
    # Configure Robot
    robot = hw.Franka(ROBOT_IP, ik)
    robot_cfg = hw.FR3Config()
    robot_cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
    robot.set_config(robot_cfg)

    # Configure Gripper
    gripper_cfg_hw = hw.FHConfig()
    gripper_cfg_hw.epsilon_inner = gripper_cfg_hw.epsilon_outer = 0.1
    gripper_cfg_hw.speed = 0.1
    gripper_cfg_hw.force = 30
    gripper = hw.FrankaHand(ROBOT_IP, gripper_cfg_hw)
    
    # Move Robot
    robot.set_cartesian_position(
        robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0.05, 0, 0]))
    )
    
    # Grasp
    gripper.grasp()
```

## CLI

The extension defines useful commands to handle the FR3 robot without the need to use the Desk Website.

```shell
python -m rcs_fr3 --help
```
