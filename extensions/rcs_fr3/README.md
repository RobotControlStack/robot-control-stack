# RCS FR3 Hardware Extension
Extension to control the fr3 with rcs.

## Installation
```shell
pip install -ve .
```

Add your FR3 credentials to a `.env` file like this:
```env
DESK_USERNAME=...
DESK_PASSWORD=...
```

## Usage
```python
import rcs_fr3
from rcs_fr3._core import hw
from rcs_fr3.desk import FCI, ContextManager, Desk, load_creds_fr3_desk
user, pw = load_creds_fr3_desk()
with FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False):
    urdf_path = rcs.scenes["fr3_empty_world"].urdf
    ik = rcs.common.RL(str(urdf_path))
    robot = hw.FR3(ROBOT_IP, ik)
    robot_cfg = FR3Config()
    robot_cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
    robot_cfg.ik_solver = IKSolver.rcs_ik
    robot.set_parameters(robot_cfg)

    gripper_cfg_hw = hw.FHConfig()
    gripper_cfg_hw.epsilon_inner = gripper_cfg_hw.epsilon_outer = 0.1
    gripper_cfg_hw.speed = 0.1
    gripper_cfg_hw.force = 30
    gripper = hw.FrankaHand(ROBOT_IP, gripper_cfg_hw)
    robot.set_cartesian_position(
        robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0.05, 0, 0]))
    )
    gripper.grasp()
```
For more examples see the [examples](../../examples/) folder.
You can switch to hardware by setting the following flag:
```python
ROBOT_INSTANCE = RobotPlatform.HARDWARE
# ROBOT_INSTANCE = RobotPlatform.SIMULATION
```


## CLI
Defines useful commands to handle the FR3 robot without the need to use the Desk Website.
You can see the available subcommands as follows:
```shell
python -m rcs_fr3 --help
```