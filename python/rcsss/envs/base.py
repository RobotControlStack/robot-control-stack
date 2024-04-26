"""Gym API."""

import logging
from enum import Enum
from typing import Any, Literal, TypedDict, cast

import gymnasium as gym
import numpy as np
from rcsss import common, hw, sim
from rcsss.camera.kinect import KinectCamera, KinectConfig

_logger = logging.getLogger(__name__)

# TODO: potential problem: on the cpp side everything is double
# TODO: potential problem: mypy and numpy types from pybindstubgen
Vec7Type = np.ndarray[Literal[7], np.dtype[np.float64]]
Vec3Type = np.ndarray[Literal[3], np.dtype[np.float64]]

RPY_SPACE = gym.spaces.Box(low=np.deg2rad(-180), high=np.deg2rad(180), shape=(3,))
XYZ_SPACE = gym.spaces.Box(low=np.array([-855, -855, 0]), high=np.array([855, 855, 1188]), shape=(3,))

POSE_SPACE = gym.spaces.Dict({"rpy": RPY_SPACE, "xyz": XYZ_SPACE})


class PoseDictType(TypedDict):
    rpy: Vec3Type
    xyz: Vec3Type


ANGLES_SPACE = gym.spaces.Box(
    low=np.array([-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895]),
    high=np.array([2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895]),
    dtype=np.float32,
    shape=(7,),
)

ARM_OBS_SPACE = gym.spaces.Dict(
    {
        "pose": POSE_SPACE,
        "angles": ANGLES_SPACE,
    }
)


class ArmObs(TypedDict):
    pose: PoseDictType
    angles: Vec7Type


# frame
COLOR_SPACE = IR_SPACE = DEPTH_SPACE = gym.spaces.Box(low=0, high=255, dtype=np.uint8)
ACC_SPACE = GYRO_SPACE = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)

# TODO: move some of this info to "info"
FRAME_SPACE = gym.spaces.Dict(
    {
        "camera": gym.spaces.Dict(
            {
                "color": COLOR_SPACE,
                "ir": IR_SPACE,
                "depth": DEPTH_SPACE,
            }
        ),
        "imu": gym.spaces.Dict(
            {
                "acc_sample": ACC_SPACE,
                "gyro_sample": GYRO_SPACE,
            }
        ),
    }
)


class CameraDictType(TypedDict):
    color: np.ndarray
    ir: np.ndarray | None
    depth: np.ndarray | None


class ImuDictType(TypedDict):
    acc_sample: np.ndarray
    gyro_sample: np.ndarray


class FrameDictType(TypedDict):
    camera: CameraDictType
    imu: ImuDictType | None


class ControlMode(Enum):
    ANGLES = 1
    CARTESIAN = 2


# def posedict_to_pose(posedict: PoseDictType) -> Pose:
#     return Pose(posedict["rpy"], posedict["xyz"])


# def pose_to_posedict(pose: Pose) -> PoseDictType:
#     return {"rpy": pose.rpy, "xyz": pose.xyz}


CartOrAngleControl = Vec7Type | PoseDictType


class FR3Env(gym.Env[ArmObs, CartOrAngleControl]):
    """Joint Gym Environment for Franka Research 3."""

    def __init__(self, robot: common.Robot, control_mode: ControlMode):
        self.robot = robot
        self.control_mode = control_mode
        self.action_space: gym.spaces.Space
        self.observation_space: gym.spaces.Space
        if control_mode == ControlMode.ANGLES:
            self.action_space = ANGLES_SPACE
        else:
            self.action_space = POSE_SPACE
        self.observation_space = ARM_OBS_SPACE

    def _get_obs(self) -> ArmObs:
        pose = self.robot.get_cartesian_position()
        rpy = pose.rotation_rpy()
        xyz = pose.translation()
        return ArmObs(
            pose=PoseDictType(rpy=np.array([rpy.roll, rpy.pitch, rpy.yaw]), xyz=np.array(xyz)),
            angles=self.robot.get_joint_position(),
        )

    def angle_step(self, act: Vec7Type) -> None:
        self.robot.set_joint_position(act)

    def cartesian_step(self, act: common.Pose) -> None:
        self.robot.set_cartesian_position(act)

    def step(self, act: CartOrAngleControl) -> tuple[ArmObs, float, bool, bool, dict]:
        if self.control_mode == ControlMode.ANGLES and isinstance(act, np.ndarray):
            self.angle_step(act)
        elif self.control_mode == ControlMode.CARTESIAN and isinstance(act, dict):
            act = cast(PoseDictType, act)
            self.cartesian_step(common.Pose(act["rpy"], act["xyz"]))
        else:
            msg = "Given type is not matching control mode!"
            raise RuntimeError(msg)
        return self._get_obs(), 0, False, False, {}

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[ArmObs, dict[str, Any]]:
        if seed is not None:
            msg = "seeding not implemented yet"
            raise NotImplementedError(msg)
        if options is not None:
            msg = "options not implemented yet"
            raise NotImplementedError(msg)
        return self._get_obs(), {}


class FR3HW(gym.Wrapper):
    def __init__(self, env: FR3Env):
        self.env: FR3Env
        super().__init__(env)
        assert isinstance(self.env.robot, hw.FR3), "Robot must be a hw.FR3 instance."
        self.hw_robot = cast(hw.FR3, self.env.robot)

    def step(self, action: CartOrAngleControl) -> tuple[ArmObs, float, bool, bool, dict]:
        try:
            return self.env.step(action)
        except hw.exceptions.FrankaControlException as e:
            _logger.error("FrankaControlException: %s", e)
            self.hw_robot.automatic_error_recovery()
            return self.env._get_obs(), 0, False, True, {}

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[ArmObs, dict[str, Any]]:
        self.hw_robot.move_home()
        return self.env.reset(seed, options)


class FR3Sim(gym.Wrapper):
    def __init__(self, env: FR3Env):
        self.env: FR3Env
        super().__init__(env)
        assert isinstance(self.env.robot, sim.FR3), "Robot must be a sim.FR3 instance."
        self.sim_robot = cast(sim.FR3, self.env.robot)

    def step(self, action: CartOrAngleControl) -> tuple[ArmObs, float, bool, bool, dict]:
        obs, _, _, _, info = self.env.step(action)
        state = self.sim_robot.get_state()
        info["collision"] = state.collision
        info["ik_success"] = state.ik_success
        # truncate episode if collision
        return obs, 0, False, state.collision or not state.ik_success, info

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[ArmObs, dict[str, Any]]:
        # TODO: reset sim
        return self.env.reset(seed, options)


KINECT_OBS_SPACE = gym.spaces.Dict(
    {
        "kinect_frame": FRAME_SPACE,
    }
)
KINECT_OBS_SPACE.spaces.update(ARM_OBS_SPACE)


class KinectObsType(ArmObs):
    kinect_frame: FrameDictType


class KinectWrapper(gym.Wrapper[KinectObsType, CartOrAngleControl, ArmObs, CartOrAngleControl]):
    def __init__(self, env: FR3HW):
        self.env: FR3HW
        self.observation_space: gym.spaces.Space
        super().__init__(env)
        config = KinectConfig()
        self.camera = KinectCamera(config)
        self.observation_space = KINECT_OBS_SPACE

    def step(self, action: CartOrAngleControl) -> tuple[KinectObsType, float, bool, bool, dict]:
        obs, reward, terminated, truncated, info = self.env.step(action)
        frame = self.camera.get_current_frame()
        kinect_frame = FrameDictType(
            camera=CameraDictType(
                color=frame.camera.color,
                ir=frame.camera.ir,
                depth=frame.camera.depth,
            ),
            imu=(
                ImuDictType(
                    acc_sample=frame.imu.acc_sample,
                    gyro_sample=frame.imu.gyro_sample,
                )
                if frame.imu is not None
                else None
            ),
        )
        kinect_obs = KinectObsType(kinect_frame=kinect_frame, angles=obs["angles"], pose=obs["pose"])
        info["camera_temperature"] = frame.camera.temperature
        if frame.imu is not None:
            info["imu_acc_sample_usec"] = frame.imu.acc_sample_usec
            info["imu_gyro_sample_usec"] = frame.imu.gyro_sample_usec
            info["imu_temperature"] = frame.imu.temperature
        return kinect_obs, reward, terminated, truncated, info


if __name__ == "__main__":
    robot = sim.FR3("models/mjcf/scene.xml", "models/urdf/fr3_from_panda.urdf", render=True)
    cfg = sim.FR3Config()
    cfg.ik_duration = 300
    cfg.realtime = True
    cfg.trajectory_trace = True
    robot.set_parameters(cfg)
    env = FR3Env(robot, ControlMode.ANGLES)
    env_sim = FR3Sim(env)
    obs, info = env_sim.reset()
    for _ in range(100):
        act = env_sim.action_space.sample()
        obs, reward, terminated, truncated, info = env_sim.step(act)
