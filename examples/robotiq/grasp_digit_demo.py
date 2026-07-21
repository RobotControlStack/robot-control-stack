import logging
from time import sleep
from typing import Any, cast

import gymnasium as gym
import mujoco
import numpy as np
from rcs._core.common import GripperType, Pose
from rcs._core.sim import SimGripperConfig, SimRobot
from rcs.envs.base import ControlMode, GripperWrapper, RelativeTo
from rcs.envs.configs import EmptyWorldFR3
from rcs_robotiq2f85.wrappers import Robotiq2F85FingerPoseWrapper
import rcs_taxim.creators
import rcs

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_NAME = "right"
ROBOTIQ_GRIPPER_TYPE = GripperType("Robotiq2F85")


def _progress(iterable):
    try:
        from tqdm import tqdm
    except ImportError:
        return iterable
    return tqdm(iterable)


def _robotiq_gripper_config(gripper_type: GripperType) -> SimGripperConfig:
    return SimGripperConfig(
        epsilon_inner=0.005,
        epsilon_outer=0.005,
        seconds_between_callbacks=0.1,
        ignored_collision_geoms=[],
        collision_geoms=[],
        collision_geoms_fingers=[],
        joints=["right_driver_joint", "left_driver_joint"],
        max_joint_width=0.005,
        min_joint_width=1.0,
        actuator="fingers_actuator",
        max_actuator_width=0,
        min_actuator_width=255,
        gripper_type=gripper_type,
    )


def create_env(gripper_type: GripperType = ROBOTIQ_GRIPPER_TYPE) -> gym.Env:
    """Create the demo using the XML registered for the configured gripper type."""
    scene = EmptyWorldFR3()
    cfg = scene.config()
    cfg.wrapper_cfg.binary_gripper = False
    gripper_cfg = _robotiq_gripper_config(gripper_type)

    cfg.control_mode = ControlMode.CARTESIAN_TRPY
    cfg.relative_to = RelativeTo.NONE
    cfg.max_relative_movement = None
    cfg.sim_cfg.realtime = True
    cfg.sim_cfg.async_control = True
    cfg.sim_cfg.frequency = 30
    cfg.robot_cfgs[ROBOT_NAME].tcp_offset = rcs.GRIPPER_TCP_OFFSETS[ROBOTIQ_GRIPPER_TYPE]
    _q_home = cfg.robot_cfgs[ROBOT_NAME].q_home.copy()
    _q_home[-1] = 0
    cfg.robot_cfgs[ROBOT_NAME].q_home = _q_home
    cfg.gripper_cfgs = {ROBOT_NAME: gripper_cfg}
    cfg.gripper_offsets = {ROBOT_NAME: rcs.GRIPPER_MOUNT_OFFSETS[ROBOTIQ_GRIPPER_TYPE]}
    cfg.camera_cfgs = None
    cfg.camera_adds = None
    cfg.root_frame_objects = {
        "": (
            rcs.OBJECT_PATHS["green_cube"],
            Pose(translation=np.array([0.5, 0.0, 0.05]), quaternion=np.array([0.0, 0.0, 0.0, 1.0])),
        )
    }

    try:
        gripper_model_path = rcs.GRIPPER_PATHS[gripper_cfg.gripper_type]
    except KeyError as exc:
        msg = f"No MuJoCo XML is registered for gripper type {gripper_cfg.gripper_type!r}"
        raise ValueError(msg) from exc

    env = scene.create_env(cfg)
    return Robotiq2F85FingerPoseWrapper(
        env,
        site_name=("left_pad_site", "right_pad_site"),
        model_path=gripper_model_path,
    )


class PickUpDemo:
    def __init__(self, env: gym.Env):
        self.env = env
        self._robot = cast(SimRobot, self.env.get_wrapper_attr("robot")[ROBOT_NAME])
        self.home_pose = self._robot.get_cartesian_position()
        self.prev_left = np.zeros(3)
        self.prev_right = np.zeros(3)
    def _action(self, pose: Pose, gripper: list[float]) -> dict[str, Any]:
        return {ROBOT_NAME: {"xyzrpy": pose.xyzrpy(), "gripper": gripper}}

    def get_object_pose(self, geom_name: str) -> Pose:
        model = self.env.get_wrapper_attr("sim").model
        data = self.env.get_wrapper_attr("sim").data

        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        obj_pose_world_coordinates = Pose(
            translation=data.geom_xpos[geom_id], rotation=data.geom_xmat[geom_id].reshape(3, 3)
        ) * Pose(rpy_vector=np.array([0, 0, 0]), translation=np.array([0.0, 0.0, 0.0]))
        return self._robot.to_pose_in_robot_coordinates(obj_pose_world_coordinates)

    def generate_waypoints(self, start_pose: Pose, end_pose: Pose, num_waypoints: int) -> list[Pose]:
        return [start_pose.interpolate(end_pose, i / num_waypoints) for i in range(num_waypoints + 1)]

    def step(self, action: dict[str, Any]) -> dict[str, Any]:
        obs = self.env.step(action)[0]
        finger_poses = obs[ROBOT_NAME][Robotiq2F85FingerPoseWrapper.FINGER_POSE_KEY]
        left_finger = finger_poses[Robotiq2F85FingerPoseWrapper.LEFT_FINGER_KEY]
        right_finger = finger_poses[Robotiq2F85FingerPoseWrapper.RIGHT_FINGER_KEY]
        left_finger_pos = left_finger[:3, 3]
        right_finger_pos = right_finger[:3, 3]
        if not(np.allclose(left_finger_pos, self.prev_left) and np.allclose(right_finger_pos, self.prev_right)):
            with np.printoptions(precision=3, suppress=True):
                print(f"{left_finger_pos}, {right_finger_pos}")
        self.prev_left = left_finger_pos
        self.prev_right = right_finger_pos
        return obs

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
        waypoints = self.plan_linear_motion(geom_name=geom_name, delta_up=0.03, num_waypoints=60)
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
    # env = create_env(gripper_type=rcs_taxim.creators.TAXIM_GRIPPER_TYPE)\
    env = create_env()
    env.get_wrapper_attr("sim").open_gui()
    sleep(3)

    for _ in _progress(range(100)):
        observation, _ = env.reset()

        controller = PickUpDemo(env)
        controller.pickup("_box_geom")
    env.close()


if __name__ == "__main__":
    main()
