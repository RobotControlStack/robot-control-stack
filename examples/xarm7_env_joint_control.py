import logging

import numpy as np
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():
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

    for _ in range(100):
        obs, info = env_rel.reset()
        for _ in range(10):
            # sample random relative action and execute it
            act = env_rel.action_space.sample()
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return


if __name__ == "__main__":
    main()
