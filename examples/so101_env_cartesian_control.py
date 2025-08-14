import logging
from time import sleep

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
    robot_cfg.robot_type = rcs.common.RobotType.SO101
    robot_cfg.attachment_site = "gripper"
    robot_cfg.arm_collision_geoms = []
    robot_cfg.mjcf_scene_path = rcs.scenes["so101_empty_world"].mjb
    robot_cfg.kinematic_model_path = rcs.scenes["so101_empty_world"].mjcf_robot


    gripper_cfg = sim.SimGripperConfig()
    gripper_cfg.min_actuator_width = -0.17453292519943295
    gripper_cfg.max_actuator_width = 1.7453292519943295
    gripper_cfg.min_joint_width = -0.17453292519943295
    gripper_cfg.max_joint_width = 1.7453292519943295
    gripper_cfg.actuator = "6"
    gripper_cfg.joint = "6"
    gripper_cfg.collision_geoms = []
    gripper_cfg.collision_geoms_fingers = []

    env_rel = SimEnvCreator()(
        robot_cfg=robot_cfg,
        control_mode=ControlMode.CARTESIAN_TQuat,
        collision_guard=False,
        gripper_cfg=gripper_cfg,
        max_relative_movement=0.5,
        relative_to=RelativeTo.LAST_STEP,
    )
    env_rel.get_wrapper_attr("sim").open_gui()
    obs, info = env_rel.reset()

    act = {"tquat": [0, 0.03, 0, 0, 0, 0, 1], "gripper": 1}
    obs, reward, terminated, truncated, info = env_rel.step(act)


    for _ in range(100):
        for _ in range(5):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0, -0.01, 0, 0, 0, 0, 1], "gripper": 0}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            print(info, obs)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
        for _ in range(5):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [0, 0.01, 0, 0, 0, 0, 1], "gripper": 1}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return


if __name__ == "__main__":
    main()
