import pickle
import time
from ctypes import c_byte, c_double, c_int
from enum import IntFlag, auto
from itertools import chain
from multiprocessing import Array, Process, RLock, Value
from multiprocessing.sharedctypes import Synchronized, SynchronizedArray
from multiprocessing.synchronize import RLock as RLockType
from queue import Queue
from socket import AF_INET, SOCK_DGRAM, socket
from struct import pack, unpack
from threading import Thread

import gymnasium as gym
import numpy as np
from numpy.typing import NDArray
from rcsss import sim
from rcsss._core.common import Pose
from rcsss.envs.base import (
    ArmObs,
    CartOrAngleControl,
    ControlMode,
    FR3Env,
    PoseDictType,
)
from rcsss.envs.sim import FR3Sim
from scipy.spatial.transform import Rotation


class Button(IntFlag):
    L_SQUEEZE = auto()
    L_TRIGGER = auto()
    R_SQUEEZE = auto()
    R_TRIGGER = auto()


class Vive(gym.Wrapper):
    def __init__(self, env: FR3Env, host: str, port: int):
        super().__init__(env)
        self._lock = RLock()
        self._pose_raw_shared = Array(c_double, 7, lock=False)
        self._buttons = Value(c_int, 0, lock=False)
        self._host = host
        self._port = port
        self._server_proc = Process(
            target=Vive.worker,
            args=(self._pose_raw_shared, self._buttons, self._lock, host, port)
        )
        self._server_proc.start()
        self.action_space = gym.Space()
        self.last_controller_pose: NDArray = np.array([0]*7, dtype=np.float64)
        self.current_controller_pose: NDArray = np.array(
            [0] * 7, dtype=np.float64)
        self.last_pose_initialized: bool = False

    def _get_obs(self) -> ArmObs:
        pose = self.env.robot.get_cartesian_position()
        rpy = pose.rotation_rpy()
        xyz = pose.translation()
        return ArmObs(
            pose=PoseDictType(rpy=np.array(
                [rpy.roll, rpy.pitch, rpy.yaw]), xyz=np.array(xyz)),
            angles=self.env.robot.get_joint_position(),
        )

    @staticmethod
    def worker(pose_raw_shared,
               buttons,
               lock: RLockType,
               host: str,
               port: int
               ):
        with socket(AF_INET, SOCK_DGRAM) as s:
            s.bind((host, port))
            while True:
                unpacked = unpack("!" + 7 * "d" + 1 * "i", s.recv(7*8+4))
                with lock:
                    pose_raw_shared[:] = unpacked[:7]
                    buttons.value = unpacked[7]

    def step(self, _):
        print("a")
        with self._lock:
            print("b")
            # if trigger not pressed return
            if not Button(self._buttons.value) & Button.R_SQUEEZE:
                self.last_pose_initialized = False
                return self._get_obs(), 0, False, False, {}
            # if last controller pose has not been initialized return
            if not self.last_pose_initialized:
                self.last_controller_pose[:] = self._pose_raw_shared[:]
                self.last_pose_initialized = True
                return self._get_obs(), 0, False, False, {}
            # Trigger pressed and self.last_controller_pose is indeed the last
            # controller pose
        with self._lock:
            new_controller_pose_view = np.ctypeslib.as_array(
                self._pose_raw_shared[:])
            rotation_new = Rotation.from_quat(new_controller_pose_view[:4])
            translation_new = new_controller_pose_view[4:7]
            rotation_old = Rotation.from_quat(self.last_controller_pose[:4])
            translation_old = self.last_controller_pose[4:7]
            rotation_diff = rotation_new * rotation_old.inv()
            translation_diff = translation_new - translation_old
            np.copyto(self.last_controller_pose, new_controller_pose_view)
        current_tcp_pose = self._get_obs()["pose"]  # type: ignore
        tmp = Pose(current_tcp_pose["rpy"], current_tcp_pose["xyz"])
        new_tcp_pose = tmp * Pose(Rotation.identity().as_euler("xyz"), translation_diff)
        act: PoseDictType = {
            "rpy": current_tcp_pose["rpy"],
            "xyz": new_tcp_pose.translation()
        }
        return self.env.step(act)


if __name__ == "__main__":
    robot = sim.FR3("models/mjcf/scene.xml",
                    "models/urdf/fr3_from_panda.urdf", render=True)
    cfg = sim.FR3Config()
    cfg.ik_duration = 300
    cfg.realtime = True
    cfg.trajectory_trace = False
    robot.set_parameters(cfg)
    env = FR3Env(robot, ControlMode.CARTESIAN)
    env_sim = FR3Sim(env)
    env_vive = Vive(env_sim, "192.168.100.1", 54321)
    obs, info = env_vive.reset()
    while True:
        obs, _, _, _, info = env_vive.step(None)
        print(obs)
        print(info)
