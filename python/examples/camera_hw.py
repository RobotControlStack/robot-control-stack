# Generic python libraries
from collections import deque
import logging
import numpy as np

# specific repo modules
import rcsss
from utils.threed_rendering import reconstruct_3d_hardware
from rcsss.envs.base import RobotInstance
from rcsss.envs.factories import (
    get_urdf_path,
    default_realsense,
)
from rcsss._core.hw import FR3Config, IKController
from rcsss.control.fr3_desk import FCI, Desk
from rcsss.control.utils import load_creds_fr3_desk

ROBOT_IPS = ["192.168.101.1", "192.168.102.1"]
ROBOT_IN_WORLD_ts = [np.eye(4), np.eye(4)]  # TODO:: get second robot transform
ROBOT_INSTANCE = RobotInstance.HARDWARE
URDF_PATH = None

# setup logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())


def main():
    # instantiate and stream hardware cameraset (realsense)
    pose_deque = deque()
    robot_to_world_deque = deque()
    cameras = {
        "wrist": "244222071045",
        "wrist_2": "243522070385",
    }
    cameras_dynamic_state = [True,True]
    camera_set = default_realsense(cameras)
    camera_set.start()
    camera_set.wait_for_frames()

    # Initialize robots
    user, pw = load_creds_fr3_desk()
    fcis = {}
    resource_mangers = []
    for i, robot_ip in enumerate(ROBOT_IPS):
        resource_manger = FCI(Desk(robot_ip, user, pw), unlock=True, lock_when_done=False)
        resource_mangers.append(resource_manger)
        with resource_manger:
            urdf_path = get_urdf_path(
                URDF_PATH, allow_none_if_not_found=False
            )  # get urdf from site packages if not provided
            ik = rcsss.common.IK(urdf_path)
            # ik = IKController.robotics_library
            robot = rcsss.hw.FR3(robot_ip, ik)
            robot_cfg = FR3Config()
            robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
            robot.set_parameters(robot_cfg)
        fcis[resource_manger] = robot
    for i, (_, robot) in enumerate(fcis.items()):
        logger.info(f"Robot {i}")
        # with resource_manager:
        try:
            pose = robot.get_cartesian_position()
        except Exception as e:
            pose = None
            logger.error(e)
            continue
        print(pose, f"pose {i}")
        pose = pose.pose_matrix()
        pose_deque.append(pose)

    robotone_to_world = np.eye(4)
    robottwo_to_world = np.eye(4)
    robot_to_world_deque.append(robotone_to_world)
    robot_to_world_deque.append(robottwo_to_world)

    pcd = reconstruct_3d_hardware(
        cameras, cameras_dynamic_state, camera_set, logger, display=True, dynamic_poses=pose_deque,
        robot_to_world_poses=robot_to_world_deque
    )
    camera_set.stop()


if __name__ == "__main__":
    main()
