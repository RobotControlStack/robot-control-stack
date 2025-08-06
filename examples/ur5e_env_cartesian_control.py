import logging

from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.creators import SimEnvCreator

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():

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
        control_mode=ControlMode.CARTESIAN_TQuat,
        collision_guard=False,
        robot_cfg=robot_cfg,
        gripper_cfg=None,
        # cameras=default_mujoco_cameraset_cfg(),
        max_relative_movement=0.5,
        relative_to=RelativeTo.LAST_STEP,
        # mjcf=rcs.scenes["ur5e_empty_world"]["mjb"],
        mjcf="/home/juelg/code/internal_rcs/assets/scenes/ur5e_empty_world/scene.xml",
        # urdf_path=rcs.scenes["ur5e_empty_world"]["urdf"],
        urdf_path="/home/juelg/code/internal_rcs/assets/ur5e/urdf/ur5e.urdf",
    )
    env_rel.get_wrapper_attr("sim").open_gui()
    env_rel.reset()

    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": 0}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": 1}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return


if __name__ == "__main__":
    main()