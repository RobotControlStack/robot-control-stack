import logging
from time import sleep

import numpy as np
from rcs._core.common import RobotPlatform
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator
from rcs_xarm7.creators import RCSXArm7EnvCreator

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.1.245"
# ROBOT_INSTANCE = RobotPlatform.SIMULATION
ROBOT_INSTANCE = RobotPlatform.HARDWARE


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        env_rel = RCSXArm7EnvCreator()(
            control_mode=ControlMode.JOINTS,
            ip=ROBOT_IP,
            relative_to=RelativeTo.LAST_STEP,
            max_relative_movement=np.deg2rad(3),
        )
    else:
        robot_cfg = sim.SimRobotConfig()
        robot_cfg.actuators = [
            "act1",
            "act2",
            "act3",
            "act4",
            "act5",
            "act6",
            "act7",
        ]
        robot_cfg.joints = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        robot_cfg.base = "base"
        robot_cfg.robot_type = rcs.common.RobotType.XArm7
        robot_cfg.attachment_site = "attachment_site"
        robot_cfg.arm_collision_geoms = []
        robot_cfg.mjcf_scene_path = rcs.scenes["xarm7_empty_world"].mjb
        robot_cfg.kinematic_model_path = rcs.scenes["xarm7_empty_world"].mjcf_robot
        env_rel = SimEnvCreator()(
            robot_cfg=robot_cfg,
            control_mode=ControlMode.JOINTS,
            collision_guard=False,
            gripper_cfg=None,
            # cameras=default_mujoco_cameraset_cfg(),
            max_relative_movement=np.deg2rad(5),
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel.get_wrapper_attr("sim").open_gui()

    with env_rel:
        for _ in range(10):
            obs, info = env_rel.reset()
            for _ in range(3):
                # sample random relative action and execute it
                act = env_rel.action_space.sample()
                print(act)
                # act["gripper"] = 1.0
                obs, reward, terminated, truncated, info = env_rel.step(act)
                print(obs)
                if truncated or terminated:
                    logger.info("Truncated or terminated!")
                    return
                # logger.info(act["gripper"])
                sleep(2.0)


if __name__ == "__main__":
    main()
