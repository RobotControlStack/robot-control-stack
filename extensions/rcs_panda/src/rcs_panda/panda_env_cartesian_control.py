import logging

from rcs.envs.base import ControlMode, RelativeTo
from rcs_panda.configs import DefaultPandaHardwareEnv

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def main():

    env = DefaultPandaHardwareEnv()
    env.ip = "192.168.4.100"
    cfg = env.config()
    cfg.control_mode = ControlMode.CARTESIAN_TQuat
    cfg.camera_cfgs = None
    cfg.max_relative_movement = 0.2
    cfg.relative_to = RelativeTo.LAST_STEP
    env_rel = env.create_env(cfg)
    input("moving")

    env_rel.reset()
    print(env_rel.get_wrapper_attr("robot").get_cartesian_position())  # type: ignore

    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": [0]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": [1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return


if __name__ == "__main__":
    main()
