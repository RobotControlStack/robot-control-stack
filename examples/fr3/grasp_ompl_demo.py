import logging
import pathlib
from typing import Any, cast

import gymnasium as gym
import mujoco
import numpy as np
from rcs._core.common import Pose, RobotType
from rcs._core.sim import SimRobot
from rcs.envs.base import ControlMode, GripperWrapper
from rcs.envs.configs import EmptyWorldFR3
from rcs.ompl.mj_ompl import MjOMPL

import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

RECORD_PATH = "/home/sbien/Documents/Development/RCS/internal_rcs/test_data"
SCENE_ROOT = pathlib.Path("~", "Documents", "Development", "RCS", "models", "scenes", "viral").expanduser()
SCENE_FILE = pathlib.Path(SCENE_ROOT, "scene.xml")
CAMERAS = [
    "wrist",
    "arro",
    "bird_eye",
    "full_side",
    "front_up",
    "front_down",
    "side_right",
    "side_left_up",
    "back_right_up",
    "back_center",
    "back_left",
    "side_opposite",
    "side_center",
]


class OmplTrajectoryDemo:
    def __init__(self, env: gym.Env, planner: MjOMPL):
        self.env = env
        self._robot = cast(SimRobot, self.env.get_wrapper_attr("robot"))
        self.home_pose: Pose = self._robot.get_cartesian_position()
        self.home_qpos: np.ndarray = self._robot.get_joint_position()
        self.sol_path = None
        self.planner = planner

    def _action(self, pose: Pose, gripper: float) -> dict[str, Any]:
        return {"xyzrpy": pose.xyzrpy(), "gripper": [gripper]}

    def _jaction(self, joints: np.ndarray, gripper: list[float]) -> dict[str, Any]:
        return {"joints": joints, "gripper": gripper}

    def get_object_pose(self, geom_name) -> Pose:
        model = self.env.get_wrapper_attr("sim").model
        data = self.env.get_wrapper_attr("sim").data

        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        obj_pose_world_coordinates = Pose(
            translation=data.geom_xpos[geom_id], rotation=data.geom_xmat[geom_id].reshape(3, 3)
        ) * Pose(rpy_vector=np.array([0, 0, np.pi]), translation=np.array([0.0, 0.0, 0.0]))
        return self._robot.to_pose_in_robot_coordinates(obj_pose_world_coordinates)

    def plan_path_to_object(self, obj_name: str, delta_up):
        self.move_home()
        obj_pose_og = self.get_object_pose(geom_name=obj_name)
        obj_pose = obj_pose_og * Pose(
            translation=np.array([0, 0, delta_up]), quaternion=np.array([1, 0, 0, 0])  # type: ignore
        )  # type: ignore
        goal_q = self.planner.ik(obj_pose, None)
        if goal_q is None:
            return False
        sol, path = self.planner.plan(goal=goal_q, start=self.home_qpos)
        if not sol:
            return False
        for waypoint in path:
            self.step(self._jaction(waypoint, GripperWrapper.BINARY_GRIPPER_OPEN))
        return True

    def approach_and_grasp(self, obj_name: str, delta_up: float = 0.2):
        obj_pose_og = self.get_object_pose(geom_name=obj_name)

        obj_pose_grasp = obj_pose_og * Pose(translation=np.array([0, 0, delta_up]), quaternion=np.array([1, 0, 0, 0]))  # type: ignore
        waypoints = self.generate_waypoints(
            start_pose=self._robot.get_cartesian_position(), end_pose=obj_pose_grasp, num_waypoints=5
        )
        for waypoint in waypoints:
            self.step(self._jaction(waypoint, GripperWrapper.BINARY_GRIPPER_OPEN))  # type: ignore
        for _ in range(3):  # Required for simulation stability
            self.step(self._jaction(waypoints[-1], GripperWrapper.BINARY_GRIPPER_CLOSED))  # type: ignore

    def generate_waypoints(self, start_pose: Pose, end_pose: Pose, num_waypoints: int) -> list[Pose]:
        start_q = self.planner.ik(start_pose, None)
        start_q = start_q[:7]  # ignore finger joints
        waypoints = [start_q]
        for i in range(num_waypoints + 1):
            t = i / (num_waypoints)
            waypoints.append(self.planner.ik(start_pose.interpolate(end_pose, t), None)[:7])  # ignore finger joints
        return waypoints

    def step(self, action: dict) -> dict:
        return self.env.step(action)[0]

    def execute_motion(self, waypoints: list[Pose], gripper: list[float] = GripperWrapper.BINARY_GRIPPER_OPEN) -> dict:
        for i in range(len(waypoints)):
            obs = self.step(self._jaction(waypoints[i], gripper))  # type: ignore
        return obs

    def move_home(self):
        end_eff_pose = self._robot.get_cartesian_position()
        waypoints = self.generate_waypoints(end_eff_pose, self.home_pose, num_waypoints=15)
        self.execute_motion(waypoints=waypoints, gripper=GripperWrapper.BINARY_GRIPPER_CLOSED)


def main():
    scene = EmptyWorldFR3()
    cfg = scene.config()
    cfg.control_mode = ControlMode.JOINTS
    cfg.sim_cfg.realtime = False
    cfg.sim_cfg.async_control = True
    cfg.max_relative_movement = None
    cfg.root_frame_objects = {
        "green_cube": (
            rcs.OBJECT_PATHS["green_cube"],
            Pose(translation=np.array([0.5, 0.0, 0.05]), quaternion=np.array([0.0, 0.0, 0.0, 1.0])),
        )
    }
    env = scene.create_env(cfg)
    env.get_wrapper_attr("sim").open_gui()
    robot_tcp = rcs.common.Pose(
        translation=np.array([0.0, 0.0, 0.1034]),  # type: ignore
        rotation=np.array([[0.707, 0.707, 0], [-0.707, 0.707, 0], [0, 0, 1]]),  # type: ignore
    )
    planner = "RRTConnect"  # one of ["PRM","RRT","RRTConnect","RRTstar","EST","FMT","BITstar"]

    for _ in range(50):
        env.reset()

        # For each episode, we create a new planner instance to reset its space information.
        ompl_planner = MjOMPL(
            robot_name="fr3",  # Name is only for logging purposes.
            robot_env=env,  # Pass the environment created above to the planner.
            njoints=7,  # Specify the number of joints
            robot_tcp=robot_tcp,  # Specify the TCP of the robot to be planned around.
            robot_xml_name=rcs.ROBOTS[RobotType.FR3].mjcf_model_path,
            robot_root_name="base_0",  # Name of the robot root body in the xml
            robot_joint_name="fr3_joint#_0",  # Joint name pattern of the robot in the xml, where # is [1~njoints]
            robot_actuator_name="fr3_joint#_0",  # Actuator name pattern, used to ensure that we only plan for actuated joints.
            obstacle_body_names=[],  # What MuJoCo bodies to consider as obstacles
            obstacle_geom_names=["floor", "box_geom"],  # What MuJoCo geoms to consider as obstacles
            interpolate_num=50,  # Number of interpolation points between start and goal configuration
        )
        ompl_planner.set_planner(planner)
        controller = OmplTrajectoryDemo(env, ompl_planner)

        # Since the target object is part of the collision to be checked, we need sufficient clearance (delta_up=0.1)
        # for the planner to find a solution.
        success = controller.plan_path_to_object("box_geom", delta_up=0.05)
        if success:
            logger.info(f"Successfully planned with {planner}.")
            # Once successful, we approach the object and grasp it.
            controller.approach_and_grasp("box_geom", delta_up=0.01)
            # And then return to home position.
            controller.move_home()
        else:
            logger.warning(f"No solution found. Failed planning with {planner}.")
            continue
        # Since we are creating the planner object at each loop to reset its space information, we delete it here.
        # This also frees the memory and prevents a potential crash.
        ompl_planner = None
    logger.info("Finished all episodes.")
    env.close()


if __name__ == "__main__":
    main()
