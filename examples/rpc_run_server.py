from rcs.envs.creators import SimEnvCreator
from rcs.envs.utils import (
	default_mujoco_cameraset_cfg,
	default_sim_gripper_cfg,
	default_sim_robot_cfg,
)
from rcs.envs.base import ControlMode, RelativeTo
from rcs.rpc.server import RcsServer

def run_server():
	env = SimEnvCreator()(
		control_mode=ControlMode.JOINTS,
		collision_guard=False,
		robot_cfg=default_sim_robot_cfg(),
		gripper_cfg=default_sim_gripper_cfg(),
		cameras=default_mujoco_cameraset_cfg(),
		max_relative_movement=0.1,
		relative_to=RelativeTo.LAST_STEP,
	)
	server = RcsServer(env, port=50051)
	server.start()
 
if __name__ == "__main__":
	run_server()
