import gymnasium as gym
import rpyc
from dataclasses import asdict
import dataclasses
from typing import Any

from matplotlib import pyplot as plt
from agents.policies import Act, Agent, Obs
import numpy as np
import rpyc
from omegaconf import OmegaConf
import json_numpy
from rcsss._core.common import Pose

import logging

logger = logging.getLogger(__name__)


class RemoteEnv:
    def __init__(self, host: str, server_port: int):
        self.server_ip = host
        self.server_port = server_port

        self.c = rpyc.connect(self.server_ip, self.server_port)

    def step(self, action):
        return json_numpy.loads(self.c.root.step(json_numpy.dumps(action)))

    def reset(self, *args, **kwargs) -> dict[str, Any]:
        # info
        return json_numpy.loads(self.c.root.reset(json_numpy.dumps((args, kwargs))))

def main():
    logging.basicConfig(level=logging.INFO)
    host = "localhost"
    port = 18861

    client = RemoteEnv(host, port)
    logger.info("Client connected to server")

    client.reset()
    logger.info("Client reset")

    act = {
        "xyzrpy": [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        "gripper": 0.0,
    }

    obs, reward, terminated, truncated, info = client.step(act)
    logger.info(f"Obs: {obs}")
    logger.info(f"Reward: {reward}")
    logger.info(f"Terminated: {terminated}")


if __name__ == "__main__":
    main()
