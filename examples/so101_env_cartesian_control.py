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
        control_mode=ControlMode.CARTESIAN_TQuat,
        collision_guard=False,
        robot_cfg=robot_cfg,
        gripper_cfg=gripper_cfg,
        # cameras=default_mujoco_cameraset_cfg(),
        max_relative_movement=0.5,
        relative_to=RelativeTo.LAST_STEP,
        mjcf=rcs.scenes["so101_empty_world"]["mjb"],
        # mjcf="/home/tobi/coding/rcs_clones/prs/assets/scenes/so101_empty_world/scene.xml", #rcs.scenes["so101_empty_world"]["mjb"],
        urdf_path=rcs.scenes["so101_empty_world"]["urdf"],
    )
    env_rel.get_wrapper_attr("sim").open_gui()
    # obs, info = env_rel.reset()
    # env_rel.get_wrapper_attr("robot").move_home()
    # env_rel.get_wrapper_attr("sim").step_until_convergence()
    # print(obs, info)
    sleep(5)
    # obs, reward, terminated, truncated, info = env_rel.step(obs)
    # print(obs, info)

    for _ in range(10):
        for _ in range(2):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.0, 0, 0.01, 0, 0, 0, 1], "gripper": 0}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            print(info, obs)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
            sleep(1)
        for _ in range(2):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [0.0, 0, -0.01, 0, 0, 0, 1], "gripper": 1}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
            sleep(1)


if __name__ == "__main__":
    main()
