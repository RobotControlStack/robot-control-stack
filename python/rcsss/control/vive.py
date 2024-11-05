import logging
import threading
import typing
from enum import IntFlag, auto
from socket import AF_INET, SOCK_DGRAM, socket
from struct import pack, unpack

import numpy as np
from rcsss._core.common import RPY, Pose
from rcsss.envs.base import (
    ControlMode,
    GripperDictType,
    LimitedTQuartRelDictType,
    RelativeActionSpace,
    RelativeTo,
    RobotInstance,
)
from rcsss.envs.factories import (
    default_fr3_hw_gripper_cfg,
    default_fr3_hw_robot_cfg,
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
    fr3_hw_env,
    fr3_sim_env,
)

# import matplotlib.pyplot as plt


logger = logging.getLogger(__name__)

EGO_LOCK = False
VIVE_HOST = "192.168.100.1"
VIVE_PORT = 54321
INCLUDE_ROTATION = True
ROBOT_IP = "192.168.101.1"
ROBOT_INSTANCE = RobotInstance.HARDWARE


class Button(IntFlag):
    L_TRIGGER = auto()
    L_SQUEEZE = auto()
    LT_CLICK = auto()
    R_TRIGGER = auto()
    R_SQUEEZE = auto()
    RT_CLICK = auto()


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
            quaternion=(self._last_controller_pose * self._offset_pose.inverse()).rotation_q(),
        )
        return (
            self._ego_transform
            * UDPViveActionServer.transform_from_openxr
            * transform
            * UDPViveActionServer.transform_from_openxr.inverse()
            * self._ego_transform.inverse()
        )

    def get_last_controller_pose(self) -> Pose:
        return self._last_controller_pose

    def run(self):
        warning_raised = False

        with socket(AF_INET, SOCK_DGRAM) as sock, socket(AF_INET, SOCK_DGRAM) as send_sock:
            sock.settimeout(2)
            sock.setblocking(False)
            sock.bind((self._host, self._port))
            send_sock.connect(("127.0.0.1", self._port + 1))
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
                        quaternion=last_controller_pose_raw[:4] if INCLUDE_ROTATION else np.array([0, 0, 0, 1]),
                    )
                    if Button(int(unpacked[7])) & self._trg_btn and not Button(int(self._buttons)) & self._trg_btn:
                        # trigger just pressed (first data sample with button pressed

                        # set forward direction based on current controller pose
                        if self._ego_lock:
                            x_axis = Pose(translation=np.array([1, 0, 0]))
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

                    elif not Button(int(unpacked[7])) & self._trg_btn and Button(int(self._buttons)) & self._trg_btn:
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
                            quaternion=(self._last_controller_pose * self._offset_pose.inverse()).rotation_q(),
                        )
                        offset = (
                            self._ego_transform
                            * UDPViveActionServer.transform_from_openxr
                            * transform
                            * UDPViveActionServer.transform_from_openxr.inverse()
                            * self._ego_transform.inverse()
                        )
                        send_sock.sendall(pack(UDPViveActionServer.FMT, *offset.rotation_q(), *offset.translation(), 0))

                    if Button(int(unpacked[7])) & self._grp_btn and not Button(int(self._buttons)) & self._grp_btn:
                        # just pressed
                        self._grp_pos = 0
                    elif not Button(int(unpacked[7])) & self._grp_btn and Button(int(self._buttons)) & self._grp_btn:
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
                LimitedTQuartRelDictType(tquart=np.concatenate([displacement.translation(), displacement.rotation_q()]))
            )
            action.update(GripperDictType(gripper=self._grp_pos))
            with self._env_lock:
                self._env.step(action)


def main():
    # if ROBOT_INSTANCE == RobotInstance.HARDWARE:
    #     user, pw = load_creds_fr3_desk()
    #     resource_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False)
    # else:
    #     resource_manger = DummyResourceManager()

    # with resource_manger:

    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        env_rel = fr3_hw_env(
            ip=ROBOT_IP,
            control_mode=ControlMode.CARTESIAN_TQuart,
            robot_cfg=default_fr3_hw_robot_cfg(),
            collision_guard="lab",
            gripper_cfg=default_fr3_hw_gripper_cfg(),
            max_relative_movement=0.5,
            relative_to=RelativeTo.CONFIGURED_ORIGIN,
        )
    else:
        env_rel = fr3_sim_env(
            control_mode=ControlMode.CARTESIAN_TQuart,
            # control_mode=ControlMode.JOINTS,
            robot_cfg=default_fr3_sim_robot_cfg(),
            gripper_cfg=default_fr3_sim_gripper_cfg(),
            camera_set_cfg=default_mujoco_cameraset_cfg(),
            max_relative_movement=0.5,
            relative_to=RelativeTo.CONFIGURED_ORIGIN,
        )

    env_rel.reset()

    with UDPViveActionServer(VIVE_HOST, VIVE_PORT, typing.cast(RelativeActionSpace, env_rel)) as action_server:
        action_server.environment_step_loop()


if __name__ == "__main__":
    main()
