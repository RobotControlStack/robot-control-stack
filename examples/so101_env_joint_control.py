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
    robot_cfg.actuators = ["1", "2", "3", "4", "5"]
    robot_cfg.joints = [
        "1",
        "2",
        "3",
        "4",
        "5",
    ]
    robot_cfg.base = "base"
    robot_cfg.robot_type = rcs.common.RobotType.UR5e
    robot_cfg.attachment_site = "gripper"
    robot_cfg.arm_collision_geoms = []
    gripper_cfg = sim.SimGripperConfig
    gripper_cfg.min_actuator_width = -0.17453292519943295
    gripper_cfg.max_actuator_width = 1.7453292519943295
    gripper_cfg.min_joint_width = -0.17453292519943295
    gripper_cfg.max_joint_width = 1.7453292519943295
    env_rel = SimEnvCreator()(
        control_mode=ControlMode.JOINTS,
        collision_guard=False,
        robot_cfg=robot_cfg,
        gripper_cfg=gripper_cfg,
        # cameras=default_mujoco_cameraset_cfg(),
        max_relative_movement=np.deg2rad(5),
        relative_to=RelativeTo.LAST_STEP,
        mjcf=rcs.scenes["so101_empty_world"]["mjb"],
        urdf_path=rcs.scenes["so101_empty_world"]["urdf"],
    )
    env_rel.get_wrapper_attr("sim").open_gui()

    for _ in range(10):
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
