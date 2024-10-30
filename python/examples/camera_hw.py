import logging
from rcsss.envs.base import ControlMode, RobotInstance
from rcsss.envs.factories import (
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
    fr3_sim_env,
    get_urdf_path,
    default_realsense,
)
from utils.camera import compute_camera_matrices, get_camera_body_transformation, get_camera_index
from utils.robot import home_robot
from utils.threed_rendering import reconstruct_3d_hardware
import open3d as o3d
import rcsss
from rcsss._core.hw import FR3Config, IKController
from rcsss.control.fr3_desk import FCI, Desk
from rcsss.control.utils import load_creds_fr3_desk

ROBOT_IP = "192.168.101.1"
# ROBOT_INSTANCE = RobotInstance.SIMULATION
ROBOT_INSTANCE = RobotInstance.HARDWARE
# replace this with a path to a robot urdf file if you dont have the utn models
URDF_PATH = None

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())


def main():
    # retrieve hardware cameraset and work with them
    # cameras = {"wrist":"244222071045", "bird-eye":"243522070364"}
    # camera_set = default_realsense({"wrist":"244222071045",
    #                                 "bird-eye":"243522070364"})
    cameras = {"wrist":"244222071045","wrist_2":"243522070385"}
    camera_set = default_realsense(cameras)
    camera_set.start()
    # print(camera_set.camera_names)
    # print(camera_set.get_depth_shape())
    camera_set.wait_for_frames()
    # frameset=camera_set.get_latest_frames()
    # depth_be = frameset.frames["bird-eye"].camera.depth.data
    # color_be = frameset.frames["bird-eye"].camera.color.data
    # camera_set.get_device_intrinsics(cameras)

    # retrieve the extrinsics of the wrist cameras relative to the ee, which are fixed from mujoco
    # cameraset_cfg = default_mujoco_cameraset_cfg()
    # cameraset_cfg.physical_units = True
    # env_rel = fr3_sim_env(
    #     control_mode=ControlMode.CARTESIAN_TQuart,
    #     robot_cfg=default_fr3_sim_robot_cfg(),
    #     gripper_cfg=default_fr3_sim_gripper_cfg(),
    #     camera_set_cfg=cameraset_cfg,
    #     max_relative_movement=0.2,
    # )
    # env_rel.reset()
    # home_robot(env_rel)
    # env_rel.step({"tquart": [-0.03, 0.07, 0.01, 0, 0, 0, 1], "gripper": 0})    # # move robot in the x direction


    # print(extrinsic_matrix,intrinsic_matrix)

    # get the point cloud from the hardware birds eye camera
    user, pw = load_creds_fr3_desk()
    resource_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=True, lock_when_done=False)
    with resource_manger:
        urdf_path = get_urdf_path(URDF_PATH, allow_none_if_not_found=False) #get urdf from site packages if not provided
        ik = rcsss.common.IK(urdf_path)
        robot = rcsss.hw.FR3(ROBOT_IP, ik)
        robot_cfg = FR3Config()
        robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
        # robot_cfg.controller = IKController.robotics_library
        robot.set_parameters(robot_cfg)
        pose=robot.get_cartesian_position()
        pose = pose.pose_matrix()
    pcd=reconstruct_3d_hardware(cameras,camera_set, logger, display=True, pose=pose)

    # pcd_filtered = pcd
    # # get pcd with x smaller than zero
    # pcd_filtered = pcd.select_by_index(np.where(np.asarray(pcd_filtered.points)[:,0]>0.05)[0])
    # # # get pcd with y smaller than zero
    # # pcd_filtered = pcd_filtered.select_by_index(np.where(np.asarray(pcd_filtered.points)[:,1]<0)[0])
    # # get pcd with z smaller than zero
    # # pcd_filtered = pcd_filtered.select_by_index(np.where(np.asarray(pcd_filtered.points)[:,2]>0)[0])
    # #show the filtered point cloud
    # o3d.visualization.draw_geometries([pcd_filtered])

    # camera_set.stop()


if __name__ == "__main__":
    main()
