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


@rpyc.service
class Server(rpyc.Service):
    def __init__(self, server_port="18861", robot_ip="192.168.103.1", robot_instance=RobotInstance.HARDWARE):
        if robot_instance == RobotInstance.HARDWARE:
            user, pw = load_creds_fr3_desk()
            resource_manger = FCI(Desk(robot_ip, user, pw), unlock=False, lock_when_done=False)
        else:
            resource_manger = DummyResourceManager()

        with resource_manger:
            if robot_instance == RobotInstance.HARDWARE:
                env_rel = fr3_hw_env(
                    ip=robot_ip,
                    control_mode=ControlMode.CARTESIAN_TRPY,
                    robot_cfg=default_fr3_hw_robot_cfg(),
                    collision_guard="lab",
                    gripper_cfg=default_fr3_hw_gripper_cfg(),
                    max_relative_movement=np.deg2rad(5),
                    relative_to=RelativeTo.CONFIGURED_ORIGIN,
                )
            else:
                env_rel = fr3_sim_env(
                    control_mode=ControlMode.CARTESIAN_TRPY,
                    collision_guard=False,
                    robot_cfg=default_fr3_sim_robot_cfg(),
                    gripper_cfg=default_fr3_sim_gripper_cfg(),
                    camera_set_cfg=default_mujoco_cameraset_cfg(),  # if this is on, it gets slow. # noqa
                    # max_relative_movement=np.deg2rad(5),
                    relative_to=RelativeTo.CONFIGURED_ORIGIN,
                )
                env_rel.get_wrapper_attr("sim").open_gui()
                self.env_rel = env_rel

        self.obs, info = self.env_rel.reset()
        print("Ready to accept commands")

    @rpyc.exposed
    def step(self, action):
        action = obtain(action)

        self.obs, reward, terminated, truncated, info = self.env_rel.step(action)


        # Deep copy to avoid issues with pickling
        self.obs = copy.deepcopy(self.obs)
        reward = copy.deepcopy(reward)
        terminated = copy.deepcopy(terminated)
        truncated = copy.deepcopy(truncated)
        info = copy.deepcopy(info)
        
        return self.obs, reward, terminated, truncated, info

    @rpyc.exposed
    def reset(self):
        self.obs, info = self.env_rel.reset()
        return self.obs, info
        #return None, None
        
    @rpyc.exposed
    def get_obs(self):
        return self.obs

    def start(self):
        t = OneShotServer(self, port=18861)
        t.start()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Starting demo")
    t = OneShotServer(Server, port=18861, protocol_config = rpyc.core.protocol.DEFAULT_CONFIG)
    t.start()
    logger.info("Server started")
    t.close()
    logger.info("Server closed")
