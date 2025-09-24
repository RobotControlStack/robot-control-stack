import logging

import numpy as np
from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs_ur5e.creators import RCSUR5eEnvCreator
from rcs_ur5e.hw import UR5eConfig

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.25.201"
ROBOT_INSTANCE = RobotPlatform.HARDWARE


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        robot_cfg = UR5eConfig()
        env_rel = RCSUR5eEnvCreator()(
            control_mode=ControlMode.JOINTS,
            robot_cfg=robot_cfg,
            ip=ROBOT_IP,
            camera_set=None,
            max_relative_movement=np.deg2rad(5),
            relative_to=RelativeTo.LAST_STEP,
        )
    else:
        robot_cfg = sim.SimRobotConfig()
        robot_cfg.actuators = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        robot_cfg.joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        robot_cfg.base = "base"
        robot_cfg.robot_type = rcs.common.RobotType.UR5e
        robot_cfg.attachment_site = "attachment_site"
        robot_cfg.arm_collision_geoms = []
        env_rel = SimEnvCreator()(
            control_mode=ControlMode.JOINTS,
            collision_guard=False,
            robot_cfg=robot_cfg,
            gripper_cfg=None,
            # cameras=default_mujoco_cameraset_cfg(),
            max_relative_movement=np.deg2rad(5),
            relative_to=RelativeTo.LAST_STEP,
            # mjcf=rcs.scenes["ur5e_empty_world"]["mjb"],
            mjcf="/home/johannes/repos/learning/robot-control-stack/assets/scenes/ur5e_empty_world/scene.xml",
            # urdf_path=rcs.scenes["ur5e_empty_world"]["urdf"],
            urdf_path="/home/johannes/repos/learning/robot-control-stack/assets/ur5e/urdf/ur5e.urdf",
        )
        env_rel.get_wrapper_attr("sim").open_gui()

    for _ in range(100):
        obs, info = env_rel.reset()
        for _ in range(3):
            # sample random relative action and execute it
            act = env_rel.action_space.sample()
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return


if __name__ == "__main__":
    main()
