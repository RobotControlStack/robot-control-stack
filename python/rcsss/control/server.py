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
                #collision_guard="lab",
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
        
        utn_gripper_offset = -0.062
        #length of the fingers in meters
        #utn_gripper_offset = 0

        # transformation: 0.2 in x direction, no rotation
        self.fingertip_T_tcp = np.array([
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,utn_gripper_offset],
            [0,0,0,1],
        ])
        logger.warning("utn finger dimensions hard coded in pyhton code!")

        self.reset()
        print("Server started")
        print("Waiting for commands ...")

    @rpyc.exposed
    def step(self, action):
        print("Executing step.")
        action = obtain(action)
        
        xyzrpy = action["xyzrpy"]
        
        requested_pose = Pose(rpy_vector=xyzrpy[3:], translation=xyzrpy[:3])
        world_T_fingertip = requested_pose.pose_matrix() # we want fingertips here
        world_T_tcp = world_T_fingertip @ self.fingertip_T_tcp # thus tcp needs to be here
        
        action["xyzrpy"] = Pose(pose_matrix=world_T_tcp).xyzrpy()
        
        # execute the action
        self.obs, reward, terminated, truncated, info = self.env_rel.step(action)
        
        obs_temp = self.get_corrected_observation(self.obs)
        
        return obs_temp, reward, terminated, truncated, info

    def get_corrected_observation(self, obs):
        # get the new pose and correct it to be in the fingertip frame
        new_xyzrpy = self.env_rel.get_obs()["xyzrpy"]
        new_pose = Pose(rpy_vector=new_xyzrpy[3:], translation=new_xyzrpy[:3])
        new_world_T_tcp = new_pose.pose_matrix() # we want fingertips here
        new_world_T_fingertip = new_world_T_tcp @ np.linalg.inv(self.fingertip_T_tcp) # thus tcp needs to be here
        
        new_xyzrpy = Pose(pose_matrix=new_world_T_fingertip).xyzrpy()
        
        obs_temp = copy.deepcopy(self.obs)
        obs_temp["xyzrpy"] = new_xyzrpy
        
        return obs_temp

    @rpyc.exposed
    def reset(self):
        print("Reseting env.")
        self.obs, info = self.env_rel.reset()
        return self.obs, info
        #return None, None
        
    @rpyc.exposed
    def get_obs(self):        
        return self.get_corrected_observation(self.obs)

    def start(self):
        import time
        while True:
            t = OneShotServer(self, port=18861)
            t.start()
            time.sleep(1)
        
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
