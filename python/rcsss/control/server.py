import rpyc

import logging
import numpy as np
from rcsss._core.common import Pose
from rcsss.control.fr3_desk import FCI, Desk, DummyResourceManager
from rcsss.control.utils import load_creds_fr3_desk
from rcsss.envs.base import ControlMode, RelativeTo, RobotInstance
from rcsss.envs.factories import (
    default_fr3_hw_gripper_cfg,
    default_fr3_hw_robot_cfg,
    default_fr3_sim_gripper_cfg,
    default_fr3_sim_robot_cfg,
    default_mujoco_cameraset_cfg,
    fr3_hw_env,
    fr3_sim_env,
)
from tqdm import tqdm
from rpyc.utils.classic import obtain
import copy

from rpyc.utils.server import ThreadedServer, OneShotServer
rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True

logger = logging.getLogger(__name__)
logger.setLevel(level=logging.INFO)


@rpyc.service
class Server(rpyc.Service):
    def __init__(self, server_port="18861", robot_ip="192.168.100.1", robot_instance=RobotInstance.HARDWARE):
        if robot_instance == RobotInstance.HARDWARE:
            user, pw = load_creds_fr3_desk()
            self.resource_manger = FCI(Desk(robot_ip, user, pw), unlock=False, lock_when_done=False)
        else:
            self.resource_manger = DummyResourceManager()
            
        self.resource_manger.__enter__()
        
        if robot_instance == RobotInstance.HARDWARE:
            env_rel = fr3_hw_env(
                ip=robot_ip,
                control_mode=ControlMode.CARTESIAN_TRPY,
                robot_cfg=default_fr3_hw_robot_cfg(),
                collision_guard="lab",
                gripper_cfg=default_fr3_hw_gripper_cfg(),
            )
        else:
            env_rel = fr3_sim_env(
                control_mode=ControlMode.CARTESIAN_TRPY,
                collision_guard=False,
                robot_cfg=default_fr3_sim_robot_cfg(),
                gripper_cfg=default_fr3_sim_gripper_cfg(),
                camera_set_cfg=default_mujoco_cameraset_cfg(),  # if this is on, it gets slow. # noqa
            )
            env_rel.get_wrapper_attr("sim").open_gui()
            
        self.env_rel = env_rel
        
        self.obs, self.info = env_rel.reset()
        print("Server started")
        print("Waiting for commands ...")

    @rpyc.exposed
    def step(self, action):
        print("Executing step.")
        action = obtain(action)
        
        self.obs, reward, terminated, truncated, info = self.env_rel.step(action)
        
        return self.obs, reward, terminated, truncated, info

    @rpyc.exposed
    def reset(self):
        print("Reseting env.")
        self.obs, info = self.env_rel.reset()
        return self.obs, info
        #return None, None
        
    @rpyc.exposed
    def get_obs(self):
        return self.obs

    def start(self):
        while True:
            t = OneShotServer(self, port=18861)
            t.start()
        
    def __del__(self):
        self.resource_manger.__exit__()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Starting demo")
    t = OneShotServer(Server, port=18861, protocol_config = rpyc.core.protocol.DEFAULT_CONFIG)
    t.start()
    logger.info("Server started")
    t.close()
    logger.info("Server closed")
