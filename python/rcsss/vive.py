
import pickle
import socket
import time
from queue import Queue
from threading import Thread

import gymnasium as gym
import numpy as np
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

MAX_UDP_SIZE = 65507


def worker(q: Queue):
    HOST, PORT = "192.168.100.1", 54321
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((HOST, PORT))
        while True:
            data, addr = s.recvfrom(MAX_UDP_SIZE)
            print(f"[WORKER THREAD] Received \n{pickle.loads(data)=}")
            if not data:
                break
            q.put(pickle.loads(data))


if __name__ == "__main__":
    robot = sim.FR3("models/mjcf/scene.xml",
                    "models/urdf/fr3_from_panda.urdf", render=True)
    cfg = sim.FR3Config()
    cfg.ik_duration = 300
    cfg.realtime = False
    cfg.trajectory_trace = True
    robot.set_parameters(cfg)
    env = FR3Env(robot, ControlMode.CARTESIAN)
    env_sim = FR3Sim(env)
    obs, info = env_sim.reset()
    queue: Queue[dict] = Queue(-1)
    thrd = Thread(target=worker, args=(queue,))
    thrd.start()
    env_sim.reset()
    while True:
        pose = queue.get(block=True)
        p: PoseDictType = {
            "rpy": np.array((pose["qx"], pose["qy"], pose["qz"])),
            "xyz": np.array((pose["x"], pose["y"], pose["z"]))
        }
        env_sim.step(p)

    # while True:
    #    obs, reward, terminated, truncated, info = env_sim.step(act)
