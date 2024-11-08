from collections import deque
import logging
import numpy as np
from rcsss.envs.base import RobotInstance
from rcsss.envs.factories import (
    get_urdf_path,
    default_realsense,
)
from utils.threed_rendering import reconstruct_3d_hardware
import rcsss
from rcsss._core.hw import FR3Config
from rcsss.control.fr3_desk import FCI, Desk
from rcsss.control.utils import load_creds_fr3_desk

ROBOT_IPS = ["192.168.101.1","192.168.102.1"]
ROBOT_IN_WORLD_ts = [np.eye(4),np.eye(4)] #TODO:: get second robot transform
ROBOT_INSTANCE = RobotInstance.HARDWARE
URDF_PATH = None

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())


def main():
    # retrieve hardware cameraset (realsense)
    pose_deque = deque()
    robot_to_world_deque = deque()
    cameras = {
        "wrist": "244222071045",
        "wrist_2": "243522070385",
    }
    cameras_dynamic_state = [True,False]
    camera_set = default_realsense(cameras)
    camera_set.start()
    camera_set.wait_for_frames()

    #Initialize robots
    robots = []
    user, pw = load_creds_fr3_desk()
    for robot_ip in ROBOT_IPS:
        resource_manger = FCI(Desk(robot_ip, user, pw), unlock=True, lock_when_done=False)
        with resource_manger:
            urdf_path = get_urdf_path(
                URDF_PATH, allow_none_if_not_found=False
            )  # get urdf from site packages if not provided
            ik = rcsss.common.IK(urdf_path)
            robot = rcsss.hw.FR3(robot_ip, ik)
            robot_cfg = FR3Config()
            robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
            robot.set_parameters(robot_cfg)
            robots.append(robot)
        for robot in robots:
            # get current robot pose to use with dynamic cameras
            pose = robot.get_cartesian_position()
            pose = pose.pose_matrix()
            pose_deque.append(pose)
        # create robot to world data, probably base on fixed measured transforms or from mujoco

        robot_to_world = np.eye(4) # stub data
        robot_to_world_deque.append(robot_to_world)

        # get second robot pose and then relate it to the first robot
    pcd = reconstruct_3d_hardware(
        cameras, cameras_dynamic_state, camera_set, logger, display=True, dynamic_poses=pose_deque,
        robot_to_world_poses=robot_to_world_deque
    )
    camera_set.stop()


if __name__ == "__main__":
    main()
