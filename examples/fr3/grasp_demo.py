import logging
from time import sleep
from typing import Any, cast

import gymnasium as gym
import mujoco
import numpy as np
from rcs._core.common import Pose
from rcs.envs.base import GripperWrapper, RobotEnv
from rcs.envs.creators import FR3SimplePickUpSimEnvCreator

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class PickUpDemo:
    def __init__(self, env: gym.Env):
        self.env = env
        self.unwrapped: RobotEnv = cast(RobotEnv, self.env.unwrapped)
        self.home_pose = self.unwrapped.robot.get_cartesian_position()

    def _action(self, pose: Pose, gripper: float) -> dict[str, Any]:
        return {"xyzrpy": pose.xyzrpy(), "gripper": gripper}

    def get_object_pose(self, geom_name) -> Pose:
        model = self.env.get_wrapper_attr("sim").model
        data = self.env.get_wrapper_attr("sim").data

        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        obj_pose_world_coordinates = Pose(
            translation=data.geom_xpos[geom_id], rotation=data.geom_xmat[geom_id].reshape(3, 3)
        ) * Pose(
            rpy_vector=np.array([0, 0, np.pi]), translation=[0, 0, 0]  # type: ignore
        )
        return self.unwrapped.robot.to_pose_in_robot_coordinates(obj_pose_world_coordinates)

    def generate_waypoints(self, start_pose: Pose, end_pose: Pose, num_waypoints: int) -> list[Pose]:
        waypoints = []
        for i in range(num_waypoints + 1):
            t = i / (num_waypoints)
            waypoints.append(start_pose.interpolate(end_pose, t))
        return waypoints

    def step(self, action: dict) -> dict:
        return self.env.step(action)[0]

    def plan_linear_motion(self, geom_name: str, delta_up: float, num_waypoints: int = 20) -> list[Pose]:
        end_eff_pose = self.unwrapped.robot.get_cartesian_position()
        goal_pose = self.get_object_pose(geom_name=geom_name)
        goal_pose *= Pose(translation=np.array([0, 0, delta_up]), quaternion=np.array([1, 0, 0, 0]))  # type: ignore
        return self.generate_waypoints(end_eff_pose, goal_pose, num_waypoints=num_waypoints)

    def execute_motion(self, waypoints: list[Pose], gripper: float = GripperWrapper.BINARY_GRIPPER_OPEN) -> dict:
        for i in range(len(waypoints)):
            obs = self.step(self._action(waypoints[i], gripper))
        return obs

    def approach(self, geom_name: str):
        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.2, num_waypoints=60)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_OPEN)

    def grasp(self, geom_name: str):

        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.01, num_waypoints=60)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_OPEN)

        self.step(self._action(Pose(), GripperWrapper.BINARY_GRIPPER_CLOSED))

        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.2, num_waypoints=60)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def move_home(self):
        end_eff_pose = self.unwrapped.robot.get_cartesian_position()
        waypoints = self.generate_waypoints(end_eff_pose, self.home_pose, num_waypoints=60)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def pickup(self, geom_name: str):
        self.approach(geom_name)
        self.grasp(geom_name)
        self.move_home()


def main():
    env = FR3SimplePickUpSimEnvCreator()(
        render_mode="human",
        delta_actions=False,
    )
    # wait for gui to open
    sleep(3)
    for _ in range(100):
        env.reset()
        print(env.unwrapped.robot.get_cartesian_position().translation())  # type: ignore
        controller = PickUpDemo(env)
        controller.pickup("box_geom")


if __name__ == "__main__":
    main()
