import logging
from time import sleep

from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs_so101.creators import RCSSO101EnvCreator, SO101SimEnvCreator

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_INSTANCE = RobotPlatform.SIMULATION


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        env_rel = RCSSO101EnvCreator()(
            id="so101_follower",
            urdf_path="/home/tobi/coding/lerobot/so101_new_calib.urdf",
            port="/dev/ttyACM0",
            calibration_dir="/home/tobi/coding/lerobot-container/calibration/robots/so101_follower/ninja_follower.json",
            # max_relative_movement=(0.5, np.deg2rad(90)),
            relative_to=RelativeTo.LAST_STEP,
        )
    else:
        cfg = sim.SimRobotConfig()
        cfg.robot_type = rcs.common.RobotType.SO101
        cfg.actuators = ["1", "2", "3", "4", "5"]
        cfg.joints = ["1", "2", "3", "4", "5"]
        cfg.arm_collision_geoms = []
        cfg.attachment_site = "gripper"

        grp_cfg = sim.SimGripperConfig()
        grp_cfg.actuator = "6"
        grp_cfg.joint = "6"
        grp_cfg.collision_geoms = []
        grp_cfg.collision_geoms_fingers = []

        env_rel = SO101SimEnvCreator()(
            control_mode=ControlMode.JOINTS,
            urdf_path="/home/tobi/coding/lerobot/so101_new_calib.urdf",
            robot_cfg=cfg,
            collision_guard=False,
            mjcf="/home/tobi/coding/lerobot/SO-ARM100/Simulation/SO101/scene.xml",
            gripper_cfg=grp_cfg,
            # camera_set_cfg=default_mujoco_cameraset_cfg(),
            max_relative_movement=None,
            # max_relative_movement=10.0,
            # max_relative_movement=0.5,
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel.get_wrapper_attr("sim").open_gui()

    for _ in range(10):
        obs, info = env_rel.reset()
        for _ in range(100):
            # sample random relative action and execute it
            act = env_rel.action_space.sample()
            print(act)
            # act["gripper"] = 1.0
            obs, reward, terminated, truncated, info = env_rel.step(act)
            print(obs)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
            logger.info(act["gripper"])
            sleep(1.0)


if __name__ == "__main__":
    main()
