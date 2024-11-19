import logging
import os
from pathlib import Path
import shutil
import tempfile
from time import sleep
from rcsss._core.common import Pose
import gymnasium as gym
import numpy as np
from rcsss._core.sim import FR3State
from rcsss.config import read_config_yaml
import xml.etree.ElementTree as ET


# from rcsss.desk import FCI, Desk
from rcsss.envs.base import ControlMode, GripperWrapper, RelativeTo
from rcsss.envs.factories import default_fr3_sim_gripper_cfg, default_fr3_sim_robot_cfg, default_mujoco_cameraset_cfg, fr3_sim_env
import json_numpy
import mujoco

from frankcsy.wrappers import DataCollector, RHCWrapper, StorageWrapper
# import utils

json_numpy.patch()

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)



class PickUpDemo:
    def __init__(self, env: gym.Env):
        self.env = env
        self.home_pose = self.env.unwrapped.robot.get_cartesian_position()

    def _action(self, pose: Pose, gripper: float) -> np.ndarray:
        return np.concatenate((pose.xyzrpy(), np.array([gripper]))) 

    def get_object_pose(self, geom_name) -> Pose:
        model = self.env.get_wrapper_attr("sim").model
        data = self.env.get_wrapper_attr("sim").data

        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        return Pose(translation=data.geom_xpos[geom_id], rotation=data.geom_xmat[geom_id].reshape(3, 3))

    
    def generate_waypoints(self, start_pose: Pose, end_pose: Pose, num_waypoints: int) -> list[Pose]:
        waypoints = []
        for i in range(num_waypoints + 1):
            t = i / (num_waypoints + 1)
            waypoints.append(start_pose.interpolate(end_pose, t))
        return waypoints

        
    def step(self, action: np.ndarray) -> dict:
        # print("Executing command")
        re = self.env.step(action)
        s: FR3State = self.env.unwrapped.robot.get_state()
        # print(f"ik success {s.ik_success}")
        return re[0]
    

    def plan_linear_motion(self, geom_name: str, delta_up: float, num_waypoints: int = 20) -> list[Pose]:
        end_eff_pose = self.env.unwrapped.robot.get_cartesian_position()

        goal_pose = self.get_object_pose(geom_name=geom_name)
        # goal pose is above the object and gripper coordinate must flip z-axis (end effector base rotation is [1, 0, 0, 0])
        # be careful we define identity quaternion as as [0, 0, 0, 1]
        # this does not work if the object is flipped
        goal_pose *= Pose(translation=[0, 0, delta_up], quaternion=[1, 0, 0, 0])

        waypoints = self.generate_waypoints(end_eff_pose, goal_pose, num_waypoints=num_waypoints)
        return waypoints
    
    def execute_motion(self, waypoints: list[Pose], gripper: float = GripperWrapper.BINARY_GRIPPER_OPEN) -> dict:
        for i in range(1, len(waypoints)):
            # calculate delta action
            delta_action = (waypoints[i] * waypoints[i - 1].inverse())
            obs = self.step(self._action(delta_action, gripper))
        return obs
    
    def approach(self, geom_name: str):
        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.2, num_waypoints=5)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_OPEN)

    def grasp(self, geom_name: str):

        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0, num_waypoints=15)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_OPEN)

        self.step(self._action(Pose(), GripperWrapper.BINARY_GRIPPER_CLOSED))

        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.2, num_waypoints=20)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def move_home(self):
        end_eff_pose = self.env.unwrapped.robot.get_cartesian_position()
        waypoints = self.generate_waypoints(end_eff_pose, self.home_pose, num_waypoints=5)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def pickup(self, geom_name: str):
        self.approach(geom_name)
        self.grasp(geom_name)
        self.move_home()


def main(storage_env: gym.Wrapper):

    # Load and parse the XML file
    xml_file = "models/scenes/fr3_empty_world/scene_openvla.xml"

    # pdb.set_trace()
    tree = ET.parse(xml_file)
    root = tree.getroot()

    pos_x = np.random.randint(30, 65) / 100.0
    pos_y = np.random.randint(-19, 20) / 100.0
    pos_z = 0.03

    new_position = f"{pos_x} {pos_y} {pos_z}"

    geom_name = "yellow_box_geom"

    # Find the joint and modify its position
    for body in root.findall('.//body'):  # Look for all body elements
        for geom in body.findall('geom'):  # Find geoms within each body
            if geom.get('name') == geom_name:
                geom.set('pos', new_position)  # Update the position
                print(f"Updated position for {geom_name} to {new_position}")


    # with tempfile.TemporaryFile() as fp:
    with tempfile.TemporaryDirectory() as tmpdir:
        scene_dir = os.path.join(tmpdir, "models")
        scene_file = os.path.join(scene_dir, "modified_scene.xml")
        shutil.copytree(Path(xml_file).parent, scene_dir)
        tree.write(scene_file)

        env = fr3_sim_env(
            control_mode=ControlMode.CARTESIAN_TRPY,
            robot_cfg=default_fr3_sim_robot_cfg(),
            gripper_cfg=default_fr3_sim_gripper_cfg(),
            camera_set_cfg=default_mujoco_cameraset_cfg(),
            max_relative_movement=(0.3, np.deg2rad(90)),
            relative_to=RelativeTo.LAST_STEP,
            mjcf=scene_file
        )

        # env.get_wrapper_attr("sim").open_gui()
        env = RHCWrapper(env, exec_horizon=1)
        # env = StorageWrapper(env, path="/home/tobi/coding/frankcsy/experiments_sim_grasping")
        storage_env.env = env

        storage_env.reset()
        controller = PickUpDemo(storage_env)

        controller.pickup(geom_name)


if __name__ == "__main__":
    dummy_env = gym.Env()
    storage_env = StorageWrapper(dummy_env, path="/home/tobi/coding/frankcsy/experiments_sim_grasping")
    with storage_env:
        for i in range(100):
            main(storage_env)
