import logging
import sys
import threading
from enum import IntFlag, auto
from socket import AF_INET, SOCK_DGRAM, socket
from struct import unpack
from time import sleep
from rcsss.camera.realsense import RealSenseCameraSet

import numpy as np
from rcsss._core.common import RPY, Pose
from rcsss.control.fr3_desk import FCI, Desk, DummyResourceManager
from rcsss.control.utils import load_creds_fr3_desk
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
    default_realsense,
    fr3_hw_env,
    fr3_sim_env,
)
from rcsss.envs.wrappers import StorageWrapper


logger = logging.getLogger(__name__)

EGO_LOCK = False
VIVE_HOST = "192.168.99.1"
VIVE_PORT = 54321

INCLUDE_ROTATION = True
ROBOT_IP = "192.168.101.1"
ROBOT_INSTANCE = RobotInstance.HARDWARE
DEBUG = True


class Button(IntFlag):
    L_TRIGGER = auto()
    L_SQUEEZE = auto()
    LT_CLICK = auto()
    R_TRIGGER = auto()
    R_SQUEEZE = auto()
    RT_CLICK = auto()


class UDPViveActionServer(threading.Thread):
    # seven doubles and one integer in network byte order
    FMT = "!" + 7 * "d" + "i" + 6 * "d"

    # base transform from OpenXR coordinate system
    # transform_from_openxr = Pose(RPY(roll=0.5 * np.pi, yaw=0)) # for second robot
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
        self._step_env = False

    def next_action(self) -> Pose:
        with self._resource_lock:
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

        with socket(AF_INET, SOCK_DGRAM) as sock:
            sock.settimeout(2)
            sock.bind((self._host, self._port))
            while not self._exit_requested:
                try:
                    unpacked = unpack(UDPViveActionServer.FMT, sock.recv(13 * 8 + 4))
                    if warning_raised:
                        logger.info("[UDP Server] connection reestablished")
                        warning_raised = False
                except TimeoutError:
                    if not warning_raised:
                        logger.warning("[UDP server] socket timeout (0.1s), waiting for packets")
                        warning_raised = True
                    continue
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
                        with self._resource_lock:
                            self._ego_transform = Pose(RPY(yaw=-rot_z))
                    else:
                        with self._resource_lock:
                            self._ego_transform = Pose()

                    with self._resource_lock:
                        self._offset_pose = last_controller_pose
                        self._last_controller_pose = last_controller_pose

                elif not Button(int(unpacked[7])) & self._trg_btn and Button(int(self._buttons)) & self._trg_btn:
                    # released
                    with self._resource_lock:
                        self._last_controller_pose = Pose()
                        self._offset_pose = Pose()
                        self._ego_transform = Pose()
                    with self._env_lock:
                        self._env.set_origin_to_current()

                elif Button(int(unpacked[7])) & self._trg_btn:
                    # button is pressed
                    with self._resource_lock:
                        self._last_controller_pose = last_controller_pose


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

    def stop_env_loop(self):
        self._step_env = False

    def environment_step_loop(self):
        self._step_env = True
        while self._step_env:
            if self._exit_requested:
                self._step_env = False
                break
            displacement = self.next_action()
            action = dict(
                LimitedTQuartRelDictType(tquart=np.concatenate([displacement.translation(), displacement.rotation_q()]))
            )

            action.update(GripperDictType(gripper=self._grp_pos))

            with self._env_lock:
                self._env.step(action)
            # rate limit
            sleep(0.001)


def input_loop(env_rel, action_server: UDPViveActionServer, camera_set: RealSenseCameraSet):
    while True:
        i = input("> ")
        match i:
            case "help":
                print("You can use `quit` to stop the program, `episode` to start a new episode")
            case "quit":
                # camera_set.stop()
                sys.exit(0)
            case "episode":
                # camera_set.clear_buffer()
                # record videos
                video_path = env_rel.path / "videos"
                video_path.mkdir(parents=True, exist_ok=True)
                print(f'{env_rel.episode_count = }')

                thread = threading.Thread(target=action_server.environment_step_loop)
                thread.start()
                input("Robot is being stepped, press enter to finish and save episode.")
                print("stopping")
                action_server.stop_env_loop()
                thread.join()
                env_rel.reset()
                print("videos saved!")


def main():
    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        user, pw = load_creds_fr3_desk()
        resource_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False, guiding_mode_when_done=True)
    else:
        resource_manger = DummyResourceManager()

    with resource_manger:

        if ROBOT_INSTANCE == RobotInstance.HARDWARE:
            camera_dict = {
                "wrist": "244222071045",
                # "wrist": "218622278131", # new realsense
                "bird_eye": "243522070364",
                # "side": "243522070385",
                "side": "244222071045",
            }
            # camera_set = default_realsense(camera_dict)
            camera_set = None
            env_rel = fr3_hw_env(
                ip=ROBOT_IP,
                # camera_set = camera_set,
                # collision_guard="lab",
                robot_cfg=default_fr3_hw_robot_cfg(),
                control_mode=ControlMode.CARTESIAN_TQuart,
                # control_mode=ControlMode.JOINTS,
                gripper_cfg=default_fr3_hw_gripper_cfg(),
                max_relative_movement=(0.05, np.deg2rad(10)),
                # TODO: max should be always according to the last step
                # max_relative_movement=np.deg2rad(20),
                relative_to=RelativeTo.CONFIGURED_ORIGIN,
                async_control=True,
            )
        else:
            env_rel = fr3_sim_env(
                control_mode=ControlMode.CARTESIAN_TQuart,
                # control_mode=ControlMode.JOINTS,
                robot_cfg=default_fr3_sim_robot_cfg(),
                collision_guard=False,
                mjcf="lab",
                gripper_cfg=default_fr3_sim_gripper_cfg(),
                # camera_set_cfg=default_mujoco_cameraset_cfg(),
                max_relative_movement=0.5,
                relative_to=RelativeTo.CONFIGURED_ORIGIN,
            )
            env_rel.get_wrapper_attr("sim").open_gui()

        if not DEBUG:
            env_rel = StorageWrapper(env_rel, path="/home/juelg/code/frankcsy/record_real_christmas", camera_set=camera_set)
            # ip_secondary = "192.168.102.1"
            # with Desk.fci(ip_secondary, user, pw):
            #     f = rcsss.hw.FR3(ip_secondary)
            #     config = rcsss.hw.FR3Config()
            #     f.set_parameters(config)
            #     env_rel.get_wrapper_attr("log_files")({
            #         "camrobot_cart.txt": str(f.get_cartesian_position()),
            #         "camrobot_joints.txt": str(f.get_joint_position()),
            #     })

        env_rel.reset()

        with env_rel:
            with UDPViveActionServer(VIVE_HOST, VIVE_PORT, env_rel) as action_server:
                if not DEBUG:
                    input_loop(env_rel, action_server, None)

                else:
                    action_server.environment_step_loop()


if __name__ == "__main__":
    main()
