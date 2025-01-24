import logging
import sys
from typing import Any, cast

import gymnasium as gym
import mujoco
import numpy as np
from rcsss._core.common import Pose
import rcsss.envs.base
from rcsss.envs.base import FR3Env, GripperWrapper

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class PickUpDemo:
    def __init__(self, env: gym.Env):
        self.env = env
        self.unwrapped: FR3Env = cast(FR3Env, self.env.unwrapped)
        self.home_pose = self.unwrapped.robot.get_cartesian_position()

    def _action(self, pose: Pose, gripper: float) -> dict[str, Any]:
        return {"xyzrpy": pose.xyzrpy(), "gripper": gripper}

    def get_object_pose(self, geom_name) -> Pose:
        model = self.env.get_wrapper_attr("sim").model
        data = self.env.get_wrapper_attr("sim").data

        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        obj_pose_world_coordinates = Pose(
            translation=data.geom_xpos[geom_id], rotation=data.geom_xmat[geom_id].reshape(3, 3)
        )
        return self.env.unwrapped.robot.to_pose_in_robot_coordinates(obj_pose_world_coordinates)

    def generate_waypoints(self, start_pose: Pose, end_pose: Pose, num_waypoints: int) -> list[Pose]:
        waypoints = []
        for i in range(num_waypoints + 1):
            t = i / (num_waypoints + 1)
            waypoints.append(start_pose.interpolate(end_pose, t))
        return waypoints

    def step(self, action: np.ndarray) -> dict:
        return self.env.step(action)

    def plan_linear_motion(self, geom_name: str, delta_up: float, num_waypoints: int = 200) -> list[Pose]:
        end_eff_pose = self.unwrapped.robot.get_cartesian_position()
        goal_pose = self.get_object_pose(geom_name=geom_name)
        goal_pose *= Pose(translation=np.array([0, 0, delta_up]), quaternion=np.array([1, 0, 0, 0]))
        return self.generate_waypoints(end_eff_pose, goal_pose, num_waypoints=num_waypoints)

    def execute_motion(self, waypoints: list[Pose], gripper: float = GripperWrapper.BINARY_GRIPPER_OPEN) -> dict:
        for i in range(1, len(waypoints)):
            # calculate delta action
            # delta_action = waypoints[i] * waypoints[i - 1].inverse()
            pose = rcsss.common.Pose(translation=waypoints[i].translation(),
                                     rpy_vector=np.array([-3.14159265e+00,  1.57009246e-16, 0]))

            # act = self._action(delta_action, gripper)
            act = self._action(pose, gripper)

            obs = self.step(act)
            ik_success = obs[-1]["ik_success"]
            if not obs[-1]["ik_success"]:
                trans_source, rot_source = waypoints[i - 1].translation(), waypoints[i - 1].rotation_rpy().as_vector()
                trans_dest, rot_des = waypoints[i].translation(), waypoints[i].rotation_rpy().as_vector()
                msg = (f"ik success: {ik_success} when attempting to move from trans: {trans_source}, rot: {rot_source}\n "
                       f"to trans: {trans_dest} rot: {rot_des}!")
                logger.warning(msg)
                assert False, msg
        return obs

    def approach(self, geom_name: str):
        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.2, num_waypoints=50)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_OPEN)

    def grasp(self, geom_name: str):

        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0, num_waypoints=50)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_OPEN)

        self.step(self._action(Pose(), GripperWrapper.BINARY_GRIPPER_CLOSED))

        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.2, num_waypoints=50)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def move_home(self):
        end_eff_pose = self.unwrapped.robot.get_cartesian_position()
        waypoints = self.generate_waypoints(end_eff_pose, self.home_pose, num_waypoints=10)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def pickup(self, geom_name: str):
        self.approach(geom_name)
        self.grasp(geom_name)
        self.move_home()


def main():
    # env = gym.make(
    #     "rcs/SimplePickUpSimDigitHand-v0",
    #     render_mode="human",
    #     delta_actions=True
    # )

    env = gym.make(
        "rcs/FR3LabPickUpSimDigitHand-v0",
        render_mode="human",
        delta_actions=False,
        robot2_cam_pose=[0.1243549, -1.4711298, 1.2246249, -1.9944441, 0.0872650, 1.3396115, 2.1275465],
    )
    env.reset()
    controller = PickUpDemo(env)
    controller.pickup("yellow_box_geom")


if __name__ == "__main__":
    main()
