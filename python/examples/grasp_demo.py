import logging

import gymnasium as gym
import mujoco
import numpy as np
from rcsss._core.common import Pose
from rcsss._core.sim import FR3State
from rcsss.envs.base import GripperWrapper

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class PickUpDemo:
    def __init__(self, env: gym.Env):
        self.env = env
        self.home_pose = self.env.unwrapped.robot.get_cartesian_position()

    def _action(self, pose: Pose, gripper: float) -> np.ndarray:
        return {"xyzrpy": pose.xyzrpy(), "gripper": gripper}

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
        re = self.env.step(action)
        s: FR3State = self.env.unwrapped.robot.get_state()
        return re[0]

    def plan_linear_motion(self, geom_name: str, delta_up: float, num_waypoints: int = 20) -> list[Pose]:
        end_eff_pose = self.env.unwrapped.robot.get_cartesian_position()
        # @todo goal_pose is with respective to the robot base which is different in empty world scene and lab scene?
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
            delta_action = waypoints[i] * waypoints[i - 1].inverse()
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
        waypoints = self.generate_waypoints(end_eff_pose, self.home_pose, num_waypoints=10)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def pickup(self, geom_name: str):
        self.approach(geom_name)
        self.grasp(geom_name)
        self.move_home()


def main():
    # available envs: "rcs/SimplePickUpSim-v0", "rcs/FR3LabPickUpSimDigitHand-v0", "SimplePickUpSimDigitHand-v0"
    env = gym.make("rcs/FR3LabPickUpSimDigitHand-v0", render_mode="human", delta_actions=True)
    env.reset()
    controller = PickUpDemo(env)
    controller.pickup("yellow_box_geom")


if __name__ == "__main__":
    main()
