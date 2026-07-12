import logging
from typing import Any, cast

import gymnasium as gym
import mujoco
import numpy as np
from rcs._core.common import Pose
from rcs._core.sim import SimRobot
from rcs.envs.base import GripperWrapper

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def _progress(iterable):
    try:
        from tqdm import tqdm
    except ImportError:
        return iterable
    return tqdm(iterable)


class PickUpDemo:
    def __init__(self, env: gym.Env):
        self.env = env
        self._robot = cast(SimRobot, self.env.get_wrapper_attr("robot")["robot"])
        self.home_pose = self._robot.get_cartesian_position()

    def _action(self, pose: Pose, gripper: list[float]) -> dict[str, Any]:
        return {"robot": {"xyzrpy": pose.xyzrpy(), "gripper": gripper}}

    def get_object_pose(self, geom_name: str) -> Pose:
        model = self.env.get_wrapper_attr("sim").model
        data = self.env.get_wrapper_attr("sim").data

        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        obj_pose_world_coordinates = Pose(
            translation=data.geom_xpos[geom_id], rotation=data.geom_xmat[geom_id].reshape(3, 3)
        ) * Pose(rpy_vector=np.array([0, 0, -np.pi / 4]), translation=np.array([0.0, 0.0, 0.0]))
        return self._robot.to_pose_in_robot_coordinates(obj_pose_world_coordinates)

    def generate_waypoints(self, start_pose: Pose, end_pose: Pose, num_waypoints: int) -> list[Pose]:
        return [start_pose.interpolate(end_pose, i / num_waypoints) for i in range(num_waypoints + 1)]

    def step(self, action: dict[str, Any]) -> dict[str, Any]:
        return self.env.step(action)[0]

    def plan_linear_motion(self, geom_name: str, delta_up: float, num_waypoints: int = 20) -> list[Pose]:
        end_eff_pose = self._robot.get_cartesian_position()
        goal_pose = self.get_object_pose(geom_name=geom_name)
        goal_pose *= Pose(translation=np.array([0, 0, delta_up]), quaternion=np.array([1, 0, 0, 0]))
        return self.generate_waypoints(end_eff_pose, goal_pose, num_waypoints=num_waypoints)

    def execute_motion(
        self, waypoints: list[Pose], gripper: list[float] = GripperWrapper.BINARY_GRIPPER_OPEN
    ) -> dict[str, Any]:
        obs: dict[str, Any] = {}
        for waypoint in waypoints:
            obs = self.step(self._action(waypoint, gripper))
        return obs

    def approach(self, geom_name: str):
        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.2, num_waypoints=60)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_OPEN)

    def grasp(self, geom_name: str):
        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.09, num_waypoints=60)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_OPEN)

        for _ in range(4):
            self.step(self._action(self._robot.get_cartesian_position(), GripperWrapper.BINARY_GRIPPER_CLOSED))

        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.2, num_waypoints=60)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def move_home(self):
        end_eff_pose = self._robot.get_cartesian_position()
        waypoints = self.generate_waypoints(end_eff_pose, self.home_pose, num_waypoints=60)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)

    def pickup(self, geom_name: str):
        self.approach(geom_name)
        self.grasp(geom_name)
        self.move_home()


def main():
    try:
        from rcs_taxim.creators import FR3TaximSimplePickUpSimEnvCreator
    except ImportError as exc:
        msg = "This example requires the rcs_taxim extension, install it with `pip install -e extensions/rcs_taxim`."
        raise ImportError(msg) from exc

    env_fact = FR3TaximSimplePickUpSimEnvCreator()
    env = env_fact(render_mode="human", delta_actions=False)

    for _ in _progress(range(100)):
        env.reset()
        controller = PickUpDemo(env)
        controller.pickup("_box_geom")
    env.close()


if __name__ == "__main__":
    main()
