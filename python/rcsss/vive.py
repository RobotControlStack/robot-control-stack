import logging
import threading
import typing
from enum import IntFlag, auto
from socket import AF_INET, SOCK_DGRAM, socket
from struct import unpack

import gymnasium as gym
import numpy as np
import rcsss
from rcsss._core.common import Pose
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    LimitedTQuartRelDictType,
    RelativeActionSpace,
    RelativeTo,
)
from rcsss.envs.sim import FR3Sim
from rcsss.sim import FR3, FR3Config, Sim

logger = logging.getLogger(__name__)


class Button(IntFlag):
    L_SQUEEZE = auto()
    L_TRIGGER = auto()
    R_SQUEEZE = auto()
    R_TRIGGER = auto()


class UDPViveActionServer(threading.Thread):
    # seven doubles and one integer in network byte order
    FMT = "!" + 7 * "d" + "i"

    def __init__(self, host: str, port: int, env: RelativeActionSpace, trg_btn=Button.R_SQUEEZE):
        super().__init__()
        self._host: str = host
        self._port: int = port

        self._resource_lock = threading.Lock()
        self._env_lock = threading.Lock()
        self._env = env
        self._trg_btn = trg_btn
        self._buttons = 0
        self._exit_requested = False
        self._last_controller_pose = Pose()
        self._offset_inverse_pose = Pose().inverse()
        self._env.set_origin_to_current()

    def next_action(self) -> Pose:
        return self._offset_inverse_pose * self._last_controller_pose

    def run(self):
        warning_raised = False
        with socket(AF_INET, SOCK_DGRAM) as sock:
            sock.settimeout(0.1)
            sock.bind((self._host, self._port))
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
                        translation=last_controller_pose_raw[4:], quaternion=last_controller_pose_raw[:4]
                    )
                    if Button(int(unpacked[7])) & self._trg_btn and not Button(int(self._buttons)) & self._trg_btn:
                        # trigger just pressed (first data sample with button pressed)
                        self._offset_inverse_pose = last_controller_pose.inverse()
                        self._last_controller_pose = last_controller_pose

                    elif not Button(int(unpacked[7])) & self._trg_btn and Button(int(self._buttons)) & self._trg_btn:
                        # released
                        with self._env_lock:
                            self._last_controller_pose = Pose()
                            self._offset_inverse_pose = Pose().inverse()
                            self._env.set_origin_to_current()

                    elif Button(int(unpacked[7])) & self._trg_btn:
                        # button is pressed
                        self._last_controller_pose = last_controller_pose

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
            action = typing.cast(
                dict,
                LimitedTQuartRelDictType(tquart=np.concat([displacement.translation(), displacement.rotation_q()])),
            )
            with self._env_lock:
                self._env.step(action)


def main():
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        return
    host = "192.168.100.1"
    port = 54321
    simulation = Sim(rcsss.scenes["fr3_empty_world"])
    robot = FR3(simulation, "0", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    fr3_config = FR3Config()
    fr3_config.realtime = False
    fr3_config.tcp_offset = Pose(quaternion=np.array([0, 0, 0, 1]), translation=np.array([0, 0, 0.1034]))
    env_sim = FR3Sim(FR3Env(robot, ControlMode.CARTESIAN_TQuart), simulation)

    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
    }
    cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=10)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam: gym.Env = CameraSetWrapper(env_sim, camera_set)

    # gripper_cfg = sim.FHConfig()
    # gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
    # env_cam = GripperWrapper(env_cam, gripper)
    env_rel = RelativeActionSpace(env_cam, relative_to=RelativeTo.CONFIGURED_ORIGIN)
    env_rel.reset()
    with UDPViveActionServer(host, port, env_rel) as action_server:
        action_server.environment_step_loop()


if __name__ == "__main__":
    main()
