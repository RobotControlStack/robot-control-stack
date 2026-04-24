import logging

import gymnasium as gym
from rcs._core.sim import SimConfig
from rcs.envs.base import (
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
    SimEnv,
)
from rcs.envs.configs import EmptyWorldSO101
from rcs.envs.sim import GripperWrapperSim, RobotSimWrapper

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():
    scene = EmptyWorldSO101()
    cfg = scene.prefixed_cfg(scene.config())
    so101 = scene.lead_robot_name(cfg)

    robot_cfg = cfg.robot_cfgs[so101]
    gripper_cfg = cfg.gripper_cfgs[so101]  # type: ignore[index]
    sim_cfg = SimConfig(
        realtime=False,
        async_control=False,
    )

    mjmodel = scene.create_model(cfg)
    simulation = sim.Sim(mjmodel, sim_cfg)

    kinematic_model_path, attachment_site = scene.kinematics_cfg(cfg)[so101]
    ik = rcs.common.Pin(
        kinematic_model_path,
        attachment_site,
    )

    robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
    env_rel: gym.Env = SimEnv(simulation)
    env_rel = RobotWrapper(env_rel, robot, ControlMode.CARTESIAN_TQuat)

    gripper = sim.SimGripper(simulation, gripper_cfg)
    env_rel = GripperWrapper(env_rel, gripper)

    env_rel = RobotSimWrapper(env_rel)
    env_rel = GripperWrapperSim(env_rel)
    env_rel = RelativeActionSpace(
        env_rel,
        max_mov=0.5,
        relative_to=RelativeTo.LAST_STEP,
    )
    env_rel = CoverWrapper(env_rel)
    env_rel.get_wrapper_attr("sim").open_gui()
    obs, info = env_rel.reset()

    act = {"tquat": [0.03, 0, 0, 0, 0, 0, 1], "gripper": [1]}
    obs, reward, terminated, truncated, info = env_rel.step(act)

    for _ in range(100):
        for _ in range(5):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0, 0, 0.01, 0, 0, 0, 1], "gripper": [0]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            print(info, obs)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
        for _ in range(5):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [0, 0, -0.01, 0, 0, 0, 1], "gripper": [1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return


if __name__ == "__main__":
    main()
