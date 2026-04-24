# RCS Panda Hardware Extension
Extension to control the panda with rcs.

## Installation
```shell
# go to this directory
sudo apt install $(cat debian_deps.txt)
pip install -ve .
```

Add your Panda credentials to a `.env` file like this:
```env
DESK_USERNAME=...
DESK_PASSWORD=...
```

## Usage
```python
import rcs_panda
from rcs_panda._core import hw
from rcs_panda.configs import DefaultPandaHardwareEnv
from rcs_panda.desk import FCI, ContextManager, Desk, load_creds_franka_desk
user, pw = load_creds_franka_desk()
with FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False):
    creator = DefaultPandaHardwareEnv()
    creator.ip = ROBOT_IP
    cfg = creator.config()
    ik = rcs.common.Pin(cfg.robot_cfg.kinematic_model_path, cfg.robot_cfg.attachment_site)
    robot = hw.Franka(cfg.robot_cfg, ik)
    gripper = hw.FrankaHand(cfg.gripper_cfg)
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
python -m rcs_panda --help
```
