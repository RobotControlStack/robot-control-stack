import os
import xml.etree.ElementTree as ET
from copy import deepcopy

import mujoco as mj
import numpy as np
from ompl import base as ob
from ompl import geometric as og
from rcs._core.common import Pose
from rcs.envs.base import RobotEnv

DEFAULT_PLANNING_TIME = 5.0  # Default time allowed for planning in seconds
INTERPOLATE_NUM = 500  # Number of points to interpolate between start and goal states


def get_collision_bodies(xml_file, robot_name):
    """
    Extract collision bodies from the XML file for a specific robot.

    Args:
        xml_file (str): Path to the XML file.
        robot_name (str): Name of the robot to search for.

    Returns:
        list: List of collision bodies' names found in the robot's subtree.
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Find the top-level body with the specified robot name
    robot = root.find(f".//body[@name='{robot_name}']")

    if robot is None:
        error_msg = f"Robot '{robot_name}' not found in the XML file."
        raise ValueError(error_msg)

    # Collect every <geom> in its subtree that has class="collision"
    collision_bodies = robot.findall(".//body")
    names = [robot_name]
    for g in collision_bodies:
        name = g.get("name")
        if name is None:
            error_msg = f"Body {ET.tostring(g).rstrip()!r}'s name is None. Please give it a name in the XML file."
            raise ValueError(error_msg)
        names.append(name)

    return names


class MjORobot:

    franka_hand_tcp = Pose(
        pose_matrix=np.array([[0.707, 0.707, 0, 0], [-0.707, 0.707, 0, 0], [0, 0, 1, 0.1034], [0, 0, 0, 1]])  # type: ignore
    )

    digit_hand_tcp = Pose(
        pose_matrix=np.array([[0.707, 0.707, 0, 0], [-0.707, 0.707, 0, 0], [0, 0, 1, 0.15], [0, 0, 0, 1]])  # type: ignore
    )

    def __init__(
        self,
        robot_name: str,
        env: RobotEnv,
        njoints: int = 7,
        robot_xml_name: str = "",
        robot_root_name: str = "base_0",
        robot_joint_name: str = "fr3_joint#_0",
        robot_actuator_name: str = "fr3_joint#_0",
        obstacle_body_names: list | None = None,
        obstacle_geom_names: list | None = None,
    ):
        """
        Initialize the robot object with the given parameters.
        It is essentially a thin wrapper around the RobotEnv (i.e. MuJoCo variables),
        for easily extracting the relevant bodies, joints, etc. from the MuJoCo model.
        Anything that directly calls MuJoCo functions should be done through this class.

        Parameters:
        - robot_name: Name of the robot.
        - env: The RobotEnv environment in which the robot operates.
        - njoints: Number of joints in the robot.
        - robot_xml_name: Path to the robot's XML file.
                          This file will be used to query the <body>s of the robot to get collision checking info.
        - robot_root_name: Name of the root body of the robot,
                           i.e. the top level <body>'s name.
        - robot_joint_name: Pattern of the robot's joints to be controlled,
                            where # will be replaced by 1 to njoints.
        - robot_actuator_name: Pattern of the robot's actuators to be controlled,
                               where # will be replaced by 1 to njoints.
                               Actuators are not controlled, but is used to validate
                               that the number of joints and actuators match.
        - obstacle_body_names: List of names of other <body>s to be checked for collision.
                               If None, it will be set to an empty list.

        """
        # First check that the xml file exists
        assert os.path.isfile(
            robot_xml_name
        ), f"Robot XML file {robot_xml_name} does not exist. Please provide a valid path."

        self.robot_name = robot_name
        self.env = env
        self.model = self.env.get_wrapper_attr("sim").model
        self.data = self.env.get_wrapper_attr("sim").data
        # Get the robot's collision geometries from the XML file
        self.robot_body_names = get_collision_bodies(robot_xml_name, robot_root_name)

        # Query the number of joints implicitly by scanning linearly from 1 to njoints
        self.joint_names = [f"{robot_joint_name.replace('#',str(i))}" for i in range(1, njoints + 1)]
        self.actuator_names = [f"{robot_actuator_name.replace('#',str(i))}" for i in range(1, njoints + 1)]

        # Grab the collision bodies, joint, and actuator IDs from the model
        self.robot_body_ids = {
            mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, name)
            for name in self.robot_body_names
            if mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, name) > -1
        }
        self.joint_ids = [
            mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
            if mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name) > -1
        ]
        self.actuator_ids = [
            mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.actuator_names
            if mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, name) > -1
        ]
        self.obstacle_body_ids: set[int] = set()
        self.obstacle_geom_ids: set[int] = set()
        self._add_obstacle_body_ids(obstacle_body_names)
        self._add_obstacle_geom_ids(obstacle_geom_names)

        # Check if the number of joints, links, and actuators match
        assert len(self.joint_ids) == len(
            self.actuator_ids
        ), f"Mismatch in number of joints and actuators for robot {robot_name}."

        self.njoints = len(self.joint_ids)

    def _remove_obstacle_body_ids(self, obstacle_body_names: list | str):
        """
        Remove specified obstacle bodies from the robot's obstacle checks, given their names.

        Args:
            obstacle_body_names (list): List of names of bodies to be removed from obstacle checks.
        """
        if obstacle_body_names is None:
            return
        obstacle_body_names = obstacle_body_names if isinstance(obstacle_body_names, list) else [obstacle_body_names]
        for name in obstacle_body_names:
            body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, name)
            if body_id in self.obstacle_body_ids:
                self.obstacle_body_ids.remove(body_id)
            else:
                error_msg = f"_remove_obstacle_body_ids: obstacle body name {name} does not exist in the set."
                raise RuntimeError(error_msg)

    def _add_obstacle_body_ids(self, obstacle_body_names: list | None):
        """
        Add specified obstacle bodies to the robot's obstacle checks, given their names.

        Args:
            obstacle_body_names (list): List of names of bodies to be added to obstacle checks.
        """
        if obstacle_body_names is None:
            return
        obstacle_body_names = obstacle_body_names if isinstance(obstacle_body_names, list) else [obstacle_body_names]
        for name in obstacle_body_names:
            body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, name)
            if body_id > -1:
                self.obstacle_body_ids.add(body_id)
            else:
                error_msg = f"_add_obstacle_body_ids: obstacle body name {name} does not exist in the model."
                raise RuntimeError(error_msg)

    def _add_obstacle_geom_ids(self, obstacle_geom_names: list | None):
        """
        Add specified obstacle geoms to the robot's obstacle checks, given their names.

        Args:
            obstacle_geom_names (list): List of names of geoms to be added to obstacle checks.
        """
        if obstacle_geom_names is None:
            return
        obstacle_geom_names = obstacle_geom_names if isinstance(obstacle_geom_names, list) else [obstacle_geom_names]
        for name in obstacle_geom_names:
            geom_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, name)
            if geom_id > -1:
                self.obstacle_geom_ids.add(geom_id)
            else:
                error_msg = f"_add_obstacle_geom_ids: obstacle geom name {name} does not exist in the model."
                raise RuntimeError(error_msg)

    def _remove_obstacle_geom_ids(self, obstacle_geom_names: list | str):
        """
        Remove specified obstacle geoms from the robot's obstacle checks, given their names.

        Args:
            obstacle_geom_names (list): List of names of geoms to be removed from obstacle checks.
        """
        if obstacle_geom_names is None:
            return
        obstacle_geom_names = obstacle_geom_names if isinstance(obstacle_geom_names, list) else [obstacle_geom_names]
        for name in obstacle_geom_names:
            geom_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, name)
            if geom_id in self.obstacle_geom_ids:
                self.obstacle_geom_ids.remove(geom_id)
            else:
                error_msg = f"_remove_obstacle_geom_ids: obstacle geom name {name} does not exist in the set."
                raise RuntimeError(error_msg)

    def check_collision(self, qpos: ob.State):
        """
        Checks for collisions when the robot is set to the given joint positions.
        This is done by setting the joint positions, running mj_fwdPosition and mj_collision,
        then resetting the simulation to the original values.

        Args:
            qpos (list): List of joint positions to set, ordered by base to end effector.
        Returns:
            bool: True if there is a collision, False otherwise.
        """
        if len(qpos) != self.njoints:
            error_msg = f"check_collision: Expected {self.njoints} joint positions, got {len(qpos)}."
            raise ValueError(error_msg)

        qpos_init = deepcopy(self.data.qpos)
        qvel_init = deepcopy(self.data.qvel)
        # Set the joint positions
        for i, joint_id in enumerate(self.joint_ids):
            self.data.qpos[joint_id] = qpos[i]

        mj.mj_fwdPosition(self.model, self.data)
        mj.mj_collision(self.model, self.data)

        has_collision = False
        for c in self.data.contact:
            b1 = self.model.geom_bodyid[c.geom1]
            b2 = self.model.geom_bodyid[c.geom2]
            # Check for collisions between robot and other bodies (first condition)
            # or self-collisions (second condition)
            if (
                (b1 in self.robot_body_ids or b2 in self.robot_body_ids)
                and (b1 in self.obstacle_body_ids or b2 in self.obstacle_body_ids)
            ) or (b1 in self.robot_body_ids and b2 in self.robot_body_ids):
                has_collision = True
                break
            # Check for collisions with geoms
            if (c.geom1 in self.obstacle_geom_ids or c.geom2 in self.obstacle_geom_ids) and (
                b1 in self.robot_body_ids or b2 in self.robot_body_ids
            ):
                has_collision = True
                break

        # Reset the simulation
        self.data.qpos = qpos_init
        self.data.qvel = qvel_init
        mj.mj_forward(self.model, self.data)
        return has_collision

    def get_joint_positions(self):
        """
        Get the current joint positions of the robot.

        Returns:
            list: Current joint positions.
        """
        # Deepcopy might not be necessary
        return [deepcopy(self.data.qpos[joint_id]) for joint_id in self.joint_ids]

    def ik(self, pose, q0=None, tcp_offset=None):
        """
        Perform inverse kinematics to find joint positions that achieve the desired pose.

        Args:
            pose (Pose): Desired end-effector pose.
            q0 (numpy.ndarray, optional): Initial guess for joint positions.
            tcp_offset (Pose, optional): Offset from the end-effector to the TCP.

        Returns:
            numpy.ndarray: Joint positions that achieve the desired pose, or None if no solution is found.
        """
        tcp_offset = tcp_offset if tcp_offset is not None else self.franka_hand_tcp
        return self.env.unwrapped.robot.get_ik().inverse(pose, q0, tcp_offset)  # type: ignore[attr-defined]


class MjOStateSpace(ob.RealVectorStateSpace):
    def __init__(self, num_dim) -> None:
        super().__init__(num_dim)
        self.num_dim = num_dim
        self.state_sampler = None

    def allocStateSampler(self):
        """
        This will be called by the internal OMPL planner
        """
        # WARN: This will cause problems if the underlying planner is multi-threaded!!!
        if self.state_sampler:
            return self.state_sampler

        # when ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()

    def set_state_sampler(self, state_sampler):
        """
        Optional, Set custom state sampler.
        """
        self.state_sampler = state_sampler


class MjOMPL:
    def __init__(
        self,
        robot_env: RobotEnv,
        robot_name: str,
        robot_xml_name: str,
        robot_root_name: str,
        robot_joint_name: str,
        robot_actuator_name: str,
        njoints: int,
        obstacle_body_names: list | None = None,
        obstacle_geom_names: list | None = None,
        interpolate_num: int = INTERPOLATE_NUM,
        default_planning_time: float = DEFAULT_PLANNING_TIME,
    ):
        """
        Initialize the OMPL planner with the given parameters.
        Besides setting up the OMPL planning context, it instantiates an MjORobot instance,
        which is a thin wrapper around the RobotEnv (i.e. MuJoCo variables),
        for easily extracting the relevant bodies, joints, etc. from the MuJoCo model.


        Parameters:
        - robot_env: The RobotEnv environment in which the robot operates.
        - robot_name: Name of the robot.

        - robot_xml_name: Path to the robot's XML file.
                          This file will be used to query the <body>s of the robot to get collision checking info.
        - robot_root_name: Name of the root body of the robot,
                           i.e. the top level <body>'s name.
        - robot_joint_name: Pattern of the robot's joints to be controlled,
                            where # will be replaced by 1 to njoints.
        - robot_actuator_name: Pattern of the robot's actuators to be controlled,
                               where # will be replaced by 1 to njoints.
                               Actuators are not controlled, but is used to validate
                               that the number of joints and actuators match.
        - njoints: Number of joints in the robot.
        - obstacle_body_names: List of names of other mjOBJ_BODYs to be checked for collision.
                               If None, it will be set to an empty list.
        - obstacle_geom_names: List of names of other mjOBJ_GEOMs to be checked for collision.
                               If None, it will be set to an empty list.
        - interpolate_num=100 (optional): Number of points to interpolate between start and goal states.
        - default_planning_time=5.0 (optional): Default time allowed for planning in seconds.

        """
        if (
            robot_name is None
            or robot_xml_name is None
            or robot_root_name is None
            or robot_joint_name is None
            or robot_actuator_name is None
            or njoints is None
        ):
            error_msg = "Initialization values are missing."
            raise ValueError(error_msg)
        # Create a new MjORobot instance
        robot = MjORobot(
            robot_name,
            robot_env,
            njoints=njoints,
            robot_xml_name=robot_xml_name,
            robot_root_name=robot_root_name,
            robot_joint_name=robot_joint_name,
            robot_actuator_name=robot_actuator_name,
            obstacle_body_names=obstacle_body_names,
            obstacle_geom_names=obstacle_geom_names,
        )

        self.robot = robot
        self.interpolate_num = interpolate_num
        self.default_planning_time = default_planning_time

        # Create the planning space
        self.space = MjOStateSpace(robot.njoints)

        # Create bounds for the space based on robot's joint limits
        bounds = ob.RealVectorBounds(robot.njoints)
        joint_bounds = [self.robot.model.jnt_range[joint_id] for joint_id in self.robot.joint_ids]
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        self.space.setBounds(bounds)

        self.ss = og.SimpleSetup(self.space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.si = self.ss.getSpaceInformation()
        self.set_planner("RRT")  # Default planner

    def is_state_valid(self, state):
        return not self.robot.check_collision(self.state_to_list(state))

    def set_planner(self, planner_name):
        """
        Note: Add your planner here!!
        """
        if planner_name == "PRM":
            self.planner = og.PRM(self.ss.getSpaceInformation())
        elif planner_name == "RRT":
            self.planner = og.RRT(self.ss.getSpaceInformation())
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.ss.getSpaceInformation())
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.ss.getSpaceInformation())
        elif planner_name == "EST":
            self.planner = og.EST(self.ss.getSpaceInformation())
        elif planner_name == "FMT":
            self.planner = og.FMT(self.ss.getSpaceInformation())
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.ss.getSpaceInformation())
        else:
            print("{} not recognized, please add it first".format(planner_name))
            return

        self.ss.setPlanner(self.planner)

    def plan(self, goal: np.ndarray, start: np.ndarray | None = None, allowed_time: float = DEFAULT_PLANNING_TIME):
        """
        Plan a path to goal from current robot state.
        Can be combined with `ik_pose` to plan a path to a desired pose.
        e.g. plan(goal=ik_pose(goal_pose), ...)
        Args:
            goal (np.ndarray): List of joint positions to reach.
            start (np.ndarray=None): List of joint positions to start from.
                             If None, it will use the current robot state.
            allowed_time (float=5.0): Time allowed for planning in seconds.
        """
        start = self.robot.get_joint_positions() if start is None else start
        return self._plan_start_goal(start, goal, allowed_time=allowed_time)

    def _plan_start_goal(self, start: np.ndarray, goal: np.ndarray, allowed_time=DEFAULT_PLANNING_TIME):
        """
        Plan a path to goal from the given robot start state.
        If a path is found, it will try to interpolate the path according to
        the number of segments specified by self.interpolate_num.

        Args:
            start (np.ndarray): List of joint positions to start from.
            goal (np.ndarray): List of joint positions to reach.
            allowed_time (float=5.0): Time allowed for planning in seconds.

        Returns:
            res (bool): True if a solution was found, False otherwise.
            sol_path_list (list[np.ndarray]): List of joint positions in the solution path, if found.
        """
        print("Planner params: \n", self.planner.params())

        # set the start and goal states;
        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]

        self.ss.setStartAndGoalStates(s, g)

        # attempt to solve the problem within allowed planning time
        solved = self.ss.solve(allowed_time)
        res = False
        sol_path_list = []
        if solved:
            print("Found solution: interpolating into {} segments".format(self.interpolate_num))
            # print the path to screen
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(self.interpolate_num)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list = [self.state_to_list(state) for state in sol_path_states]
            for sol_path in sol_path_list:
                self.is_state_valid(sol_path)
            res = True
        else:
            print("No solution found")

        # reset robot state
        # self.robot.set_state(orig_robot_state)
        sol_path_list = [np.array(sol_path, dtype=np.float32) for sol_path in sol_path_list]
        return res, sol_path_list

    def ik(self, pose: Pose, q0: np.ndarray | None = None, tcp_offset: Pose | None = None):
        """
        Perform inverse kinematics to find joint positions that achieve the desired pose.

        Args:
            pose (Pose): Desired end-effector pose.
            q0 (numpy.ndarray, optional): Initial guess for joint positions.
            tcp_offset (Pose, optional): Offset from the end-effector to the TCP.
                    if not provided, it will use the default Franka Hand TCP pose.

        Returns:
            numpy.ndarray: Joint positions that achieve the desired pose, or None if no solution is found.
        """
        if q0 is None:
            q0 = self.robot.get_joint_positions()
        return self.robot.ik(pose, q0, tcp_offset)

    def state_to_list(self, state):
        """
        Convert an OMPL state to a list of joint positions.

        Args:
            state (ob.State): The OMPL state to convert.
        Returns:
            list: List of joint positions corresponding to the state.
        """
        return [state[i] for i in range(self.robot.njoints)]

    def set_state_sampler(self, state_sampler):
        self.space.set_state_sampler(state_sampler)

    def add_collision_bodies(self, obstacle_body_names: list):
        """
        Add specified obstacle bodies to the robot's obstacle checks.
        Prints a warning if the body name does not exist in the model.

        Args:
            obstacle_body_names (list|str): List of names of bodies to be added to obstacle checks.
        """
        self.robot._add_obstacle_body_ids(obstacle_body_names)

    def add_collision_geoms(self, obstacle_geom_names: list):
        """
        Add specified obstacle geometries to the robot's obstacle checks.
        Prints a warning if the geometry name does not exist in the model.

        Args:
            obstacle_geom_names (list|str): List of names of geometries to be added to obstacle checks.
        """
        self.robot._add_obstacle_geom_ids(obstacle_geom_names)

    def remove_collision_bodies(self, obstacle_body_names: list | str):
        """
        Remove specified obstacle bodies from the robot's obstacle checks.
        Prints a warning if the body name does not exist in the current set of obstacles.

        Args:
            obstacle_body_names (list|str): List of names of bodies to be removed from obstacle checks.
        """
        self.robot._remove_obstacle_body_ids(obstacle_body_names)

    def remove_collision_geoms(self, obstacle_geom_names: list | str):
        """
        Remove specified obstacle geometries from the robot's obstacle checks.
        Prints a warning if the geometry name does not exist in the current set of obstacles.

        Args:
            obstacle_geom_names (list|str): List of names of geometries to be removed from obstacle checks.
        """
        self.robot._remove_obstacle_geom_ids(obstacle_geom_names)
