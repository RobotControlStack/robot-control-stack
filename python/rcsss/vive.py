import logging
import threading
import typing
from enum import IntFlag, auto
from socket import AF_INET, SOCK_DGRAM, socket
from struct import unpack  # , pack

import gymnasium as gym
import numpy as np
import rcsss
from rcsss._core.common import RPY, Pose
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.config import read_config_yaml
from rcsss.desk import FCI, Desk
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperDictType,
    GripperWrapper,
    LimitedTQuartRelDictType,
    RelativeActionSpace,
    RelativeTo,
)
from rcsss.envs.hw import FR3HW
from rcsss.envs.sim import CollisionGuard, FR3Sim
from rcsss.sim import FR3, FR3Config, Sim

# import matplotlib.pyplot as plt


logger = logging.getLogger(__name__)

EGO_LOCK = False
VIVE_HOST = "192.168.100.1"
VIVE_PORT = 54321
USE_REAL_ROBOT = False
INCLUDE_ROTATION = False
ROBOT_IP = "192.168.101.1"


class Button(IntFlag):
    L_TRIGGER = auto()
    L_SQUEEZE = auto()
    R_TRIGGER = auto()
    R_SQUEEZE = auto()


class UDPViveActionServer(threading.Thread):
    # seven doubles and one integer in network byte order
    FMT = "!" + 7 * "d" + "i"

    # base transform from OpenXR coordinate system
    transform_from_openxr = Pose(RPY(roll=0.5 * np.pi, yaw=np.pi))

    def __init__(
        self, host: str, port: int, env: RelativeActionSpace, trg_btn=Button.R_TRIGGER, grp_btn=Button.R_SQUEEZE
    ):
        super().__init__()
        self._host: str = host
        self._port: int = port

        self._resource_lock = threading.Lock()
        self._env_lock = threading.Lock()
        self._env = env
        self._trg_btn = trg_btn
        self._grp_btn = grp_btn
        self._grp_pos = 1
        self._buttons = 0
        self._exit_requested = False
        self._last_controller_pose = Pose()
        self._offset_pose = Pose()
        self._ego_lock = EGO_LOCK
        self._ego_transform = Pose()
        self._env.set_origin_to_current()

    def next_action(self) -> Pose:
        transform = Pose(
            translation=self._last_controller_pose.translation() - self._offset_pose.translation(),
            quaternion=(self._offset_pose.inverse() * self._last_controller_pose).rotation_q(),
        )
        return (
            self._ego_transform
            * UDPViveActionServer.transform_from_openxr
            * transform
            * UDPViveActionServer.transform_from_openxr.inverse()
            * self._ego_transform.inverse()
        )

    def get_last_controller_pose(self) -> Pose:
        return self._last_controller_pose()

    def run(self):
        warning_raised = False

        with socket(AF_INET, SOCK_DGRAM) as sock:
            with socket(AF_INET, SOCK_DGRAM) as send_sock:
                sock.settimeout(2)
                sock.bind((self._host, self._port))
                # send_sock.connect(("127.0.0.1", self._port + 1))
                while not self._exit_requested:
                    try:
                        unpacked = unpack(UDPViveActionServer.FMT, sock.recv(7 * 8 + 4))
                        if warning_raised:
                            logger.info("[UDP Server] connection reestablished")
                            warning_raised = False
                    except TimeoutError:
                        if not warning_raised:
                            logger.warning("[UDP server] socket timeout (0.1s), waiting for packets")
                            warning_raised = True
                        continue
                    with self._resource_lock:
                        last_controller_pose_raw = np.ctypeslib.as_array(unpacked[:7])
                        last_controller_pose = Pose(
                            translation=last_controller_pose_raw[4:],
                            quaternion=last_controller_pose_raw[:4] if INCLUDE_ROTATION else [0, 0, 0, 1],
                        )
                        if Button(int(unpacked[7])) & self._trg_btn and not Button(int(self._buttons)) & self._trg_btn:
                            # trigger just pressed (first data sample with button pressed

                            # set forward direction based on current controller pose
                            if self._ego_lock:
                                x_axis = Pose(translation=[1, 0, 0])
                                x_axis_rot = (
                                    UDPViveActionServer.transform_from_openxr
                                    * Pose(quaternion=last_controller_pose.rotation_q())
                                    * UDPViveActionServer.transform_from_openxr.inverse()
                                    * x_axis
                                )

                                # Compute angle around z axis: https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
                                rot_z = np.atan2(x_axis_rot.translation()[1], x_axis_rot.translation()[0]) - np.atan2(
                                    x_axis.translation()[1], x_axis.translation()[0]
                                )
                                rot_z -= np.pi / 2

                                print(f"Angle: {rot_z*180/np.pi}")
                                self._ego_transform = Pose(RPY(yaw=-rot_z))
                            else:
                                self._ego_transform = Pose()

                            self._offset_pose = last_controller_pose
                            self._last_controller_pose = last_controller_pose

                        elif (
                            not Button(int(unpacked[7])) & self._trg_btn and Button(int(self._buttons)) & self._trg_btn
                        ):
                            # released
                            with self._env_lock:
                                self._last_controller_pose = Pose()
                                self._offset_pose = Pose()
                                self._ego_transform = Pose()
                                self._env.set_origin_to_current()

                        elif Button(int(unpacked[7])) & self._trg_btn:
                            # button is pressed
                            self._last_controller_pose = last_controller_pose

                            # plot current offset with liveplot.py
                            transform = Pose(
                                translation=self._last_controller_pose.translation() - self._offset_pose.translation(),
                                quaternion=(self._offset_pose.inverse() * self._last_controller_pose).rotation_q(),
                            )
                            offset = (
                                self._ego_transform
                                * UDPViveActionServer.transform_from_openxr
                                * transform
                                * UDPViveActionServer.transform_from_openxr.inverse()
                                * self._ego_transform.inverse()
                            )
                            # send_sock.sendall(pack(UDPViveActionServer.FMT, *offset.rotation_q(), *offset.translation(), 0))

                        if Button(int(unpacked[7])) & self._grp_btn and not Button(int(self._buttons)) & self._grp_btn:
                            # just pressed
                            self._grp_pos = 0
                        elif (
                            not Button(int(unpacked[7])) & self._grp_btn and Button(int(self._buttons)) & self._grp_btn
                        ):
                            # just released
                            self._grp_pos = 1

                        self._buttons = unpacked[7]

    def stop(self):
        self._exit_requested = True
        self.join()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()

    def environment_step_loop(self):
        while True:
            with self._resource_lock:
                displacement = self.next_action()
            action = dict(
                LimitedTQuartRelDictType(tquart=np.concat([displacement.translation(), displacement.rotation_q()]))
            )
            action.update(GripperDictType(gripper=self._grp_pos))
            with self._env_lock:
                self._env.step(action)


def hw():

    cfg = read_config_yaml("config.yaml")
    d = Desk(ROBOT_IP, cfg.hw.username, cfg.hw.password)
    with FCI(d, unlock=False, lock_when_done=False):

        robot = rcsss.hw.FR3(ROBOT_IP, str(rcsss.scenes["lab"].parent / "fr3.urdf"))
        rcfg = rcsss.hw.FR3Config()
        rcfg.tcp_offset = rcsss.common.FrankaHandTCPOffset()
        rcfg.speed_factor = 0.2
        # rcfg.controller = rcsss.hw.IKController.robotics_library
        robot.set_parameters(rcfg)

        # env = FR3Env(robot, ControlMode.CARTESIAN_TQuart)
        env = FR3Env(robot, ControlMode.JOINTS)
        env_hw: gym.Env = FR3HW(env)
        gripper_cfg = rcsss.hw.FHConfig()
        gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.5
        gripper_cfg.speed = 0.1
        gripper_cfg.force = 30
        gripper = rcsss.hw.FrankaHand(ROBOT_IP, gripper_cfg)
        # gripper.homing()
        env_hw = GripperWrapper(env_hw, gripper, binary=True)

        # TODO: camera
        env_hw: gym.Env = CollisionGuard.env_from_xml_paths(
            env_hw,
            str(rcsss.scenes["fr3_empty_world"]),
            str(rcsss.scenes["lab"].parent / "fr3.urdf"),
            gripper=True,
            check_home_collision=False,
            camera=True,
            control_mode=ControlMode.CARTESIAN_TQuart,
            tcp_offset=rcsss.common.FrankaHandTCPOffset(),
        )

        env_rel = RelativeActionSpace(env_hw, relative_to=RelativeTo.CONFIGURED_ORIGIN)
        env_rel.reset()
        with UDPViveActionServer(VIVE_HOST, VIVE_PORT, env_rel) as action_server:
            action_server.environment_step_loop()


def sim():
    simulation = Sim(rcsss.scenes["fr3_empty_world"])
    robot = FR3(simulation, "0", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    fr3_config = FR3Config()
    fr3_config.realtime = False
    # TODO: We might need a TCP offset with only translation here
    env_sim = FR3Sim(FR3Env(robot, ControlMode.JOINTS), simulation)

    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
    }
    cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=10)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam: gym.Env = CameraSetWrapper(env_sim, camera_set)

    gripper_cfg = rcsss.sim.FHConfig()
    gripper = rcsss.sim.FrankaHand(simulation, "0", gripper_cfg)
    env_cam = GripperWrapper(env_cam, gripper)

    env_cam = CollisionGuard.env_from_xml_paths(
        env_cam,
        str(rcsss.scenes["fr3_empty_world"]),
        str(rcsss.scenes["lab"].parent / "fr3.urdf"),
        gripper=True,
        camera=False,
        check_home_collision=False,
        control_mode=ControlMode.CARTESIAN_TQuart,
    )
    env_rel = RelativeActionSpace(env_cam, relative_to=RelativeTo.CONFIGURED_ORIGIN)
    env_rel.reset()
    with UDPViveActionServer(VIVE_HOST, VIVE_PORT, env_rel) as action_server:
        action_server.environment_step_loop()


def main():
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        return
    if USE_REAL_ROBOT:
        hw()
    else:
        sim()


if __name__ == "__main__":
    main()
