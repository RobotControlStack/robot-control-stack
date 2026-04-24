import numpy as np
from rcs.envs.base import ControlMode, RelativeTo
from rcs.envs.configs import EmptyWorldFR3
from rcs.rpc.server import RcsServer


def run_server():
    scene = EmptyWorldFR3()
    cfg = scene.config()
    cfg.control_mode = ControlMode.JOINTS
    cfg.max_relative_movement = np.deg2rad(5)
    cfg.relative_to = RelativeTo.LAST_STEP
    env = scene.create_env(cfg)
    server = RcsServer(env, port=50051)
    server.start()


if __name__ == "__main__":
    run_server()
