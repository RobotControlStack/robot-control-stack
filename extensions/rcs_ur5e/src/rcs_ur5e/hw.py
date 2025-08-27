import typing
from dataclasses import dataclass
import multiprocessing as mp
from multiprocessing.shared_memory import SharedMemory
import time
import numpy as np
import rtde_control
import rtde_receive
from rcs import common
from rcs_ur5e import robotiq_gripper


@dataclass(kw_only=True)
class UR5eConfig(common.RobotConfig):
    lookahead_time: float = 0.05
    gain: float = 500.0
    max_velocity: float = 1.0
    max_acceleration: float = 1.0
    async_control: bool = True
    max_servo_joint_step: float = 0.15
    max_servo_cartesian_step: float = 0.01

    def __post_init__(self):
        super().__init__()


@dataclass(kw_only=True)
class UR5eState(common.RobotState):
    def __post_init__(self):
        super().__init__()


# Define the shared memory size and name outside the class
SHM_SIZE = 4 + 1 + 48 + 48 + 48 + 48
SHM_NAME = "ur5e_control_shm"

NO_MODE = 0
JOINT_MODE = 1
CARTESIAN_MODE = 2

def _control_robot(shm_name: str, ip: str, stop_queue: mp.Queue, config_queue: mp.Queue) -> None:
    """
    Control loop for the robot, running in a separate process.
    This function is a helper and not part of the class.
    """
    robot_config = UR5eConfig()
    try:
        # Initialize robot interfaces
        # TODO(j.hechtl): this is currently blocking if connection fails
        ur_control = rtde_control.RTDEControlInterface(ip)
        ur_receive = rtde_receive.RTDEReceiveInterface(ip)

        if not ur_control.isConnected():
            raise ConnectionError(f"Could not connect to UR5e at {ip}.")

        # Attach to the shared memory segment
        shm = SharedMemory(name=shm_name)
        data_buffer = shm.buf
        
        # Define offsets for each field in the shared memory buffer
        offset_mode = 0
        offset_target_reached = offset_mode + 4
        offset_joint_target = offset_target_reached + 1
        offset_cartesian_target = offset_joint_target + 48
        offset_joint_state = offset_cartesian_target + 48
        offset_cartesian_state = offset_joint_state + 48

        # Create numpy views on the shared memory buffer
        joint_target_view = np.ndarray(
            (6,), dtype=np.float64, buffer=data_buffer, offset=offset_joint_target
        )
        cartesian_target_view = np.ndarray(
            (6,), dtype=np.float64, buffer=data_buffer, offset=offset_cartesian_target
        )
        joint_state_view = np.ndarray(
            (6,), dtype=np.float64, buffer=data_buffer, offset=offset_joint_state
        )
        cartesian_state_view = np.ndarray(
            (6,), dtype=np.float64, buffer=data_buffer, offset=offset_cartesian_state
        )

        print("Robot control process started.")

        dt = 1.0/500.0  # 2ms

        while stop_queue.empty():
            if not config_queue.empty():
                robot_config = config_queue.get()
            t_start = ur_control.initPeriod()
            mode = int.from_bytes(data_buffer[offset_mode:offset_target_reached], "little")

            joint_state = np.array(ur_receive.getActualQ())
            joint_state_view[:] = joint_state
            
            cartesian_state = np.array(ur_receive.getActualTCPPose())
            cartesian_state_view[:] = cartesian_state

            if mode == JOINT_MODE: 
                diff = joint_target_view - joint_state_view
                if np.max(np.abs(diff)) < 0.01:
                    data_buffer[offset_target_reached] = 1

                if np.max(np.abs(diff)) > robot_config.max_servo_joint_step:
                    diff = (robot_config.max_servo_joint_step / np.max(np.abs(diff))) * diff
                    target_q = (joint_state_view + diff).tolist()
                else:
                    target_q = joint_target_view.tolist()

                # current_qd = ur_receive.getActualQd()
                # required_qd = diff / dt 
                # required_qdd = (required_qd - current_qd) / dt

                # max_qdd = [1,1,1,1,1,1]

                # factor = 1

                # for i in range(len(max_qdd)):
                #     if np.abs(required_qdd[i])*factor > max_qdd[i]:
                #         factor = factor * (max_qdd[i]/np.abs(required_qdd))
                # assert factor <= 1
                # if factor < 1:
                #     target_qdd = factor * required_qdd
                #     target_qd = current_qd + 0.5 * dt *target_qdd # average velocity in time window
                #     new_target_q = joint_state + target_qd * dt
                #     print("Current q: ", joint_state, " Target q: ", target_q, " New: ", new_target_q)

                # print("Servo")
                ur_control.servoJ(
                    target_q,
                    robot_config.max_velocity,
                    robot_config.max_acceleration,
                    dt,
                    robot_config.lookahead_time,
                    robot_config.gain,
                )
                
            elif mode == CARTESIAN_MODE: 
                rotvec = common.RotVec(np.array(cartesian_target_view[3:6]))
                # print("Target View: ", cartesian_target_view)
                a = common.Pose(quaternion=rotvec.as_quaternion_vector(), translation=cartesian_target_view[:3])
                # print(a)
                rotvec = common.RotVec(np.array(cartesian_state_view[3:6]))
                b = common.Pose(quaternion=rotvec.as_quaternion_vector(), translation=cartesian_state_view[:3])
                # print(b)
                diff = b * a.inverse()
                # print(diff)
                diff.limit_rotation_angle(np.deg2rad(0.01)).limit_translation_length(0.008)
                target = a * diff
                # print("Target: ",target)
                target_pose = target.rotvec()

                diff = cartesian_target_view - cartesian_state_view
                if np.max(np.abs(diff)) < 0.0025:
                    data_buffer[offset_target_reached] = 1

                # if np.max(np.abs(diff[0:3])) > robot_config.max_servo_cartesian_step:
                #     diff = (robot_config.max_servo_cartesian_step / np.max(diff)) * diff
                #     target_pose = (cartesian_state_view + diff).tolist()
                # else:
                # target_pose = cartesian_target_view.tolist()
                ur_control.servoL(
                    target_pose,
                    robot_config.max_velocity,
                    robot_config.max_acceleration,
                    dt,
                    robot_config.lookahead_time,
                    robot_config.gain,
                )

            ur_control.waitPeriod(t_start)
                
    except Exception as e:
        print(f"Robot control process encountered an error: {e}")
        
    finally:
        print("Robot control process is shutting down...")
        if 'ur_control' in locals():
            ur_control.servoStop()
            ur_control.stopScript()
            ur_control.disconnect()
        if 'shm' in locals():
            shm.close()

class UR5e: # (common.Robot): # should inherit and implement common.Robot, but currently there is a bug that needs to be fixed
    def __init__(self, ip: str):
        self.ik = common.RL(urdf_path="/home/johannes/repos/rcs/assets/ur5e/urdf/ur5e.urdf")
        self._config = UR5eConfig()
        self._config.robot_type = common.RobotType.UR5e
        self._ip = ip

        # Initialize shared memory and communication queues
        self._shm = SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
        self._shm_buffer = self._shm.buf
        self._stop_queue = mp.Queue()
        self._config_queue = mp.Queue()

        # Define numpy views on the shared memory buffer
        self._offset_mode = 0
        self._offset_target_reached = self._offset_mode + 4
        self._offset_joint_target = self._offset_target_reached + 1
        self._offset_cartesian_target = self._offset_joint_target + 48
        self._offset_joint_state = self._offset_cartesian_target + 48
        self._offset_cartesian_state = self._offset_joint_state + 48

        self._joint_target_shm = np.ndarray((6,), dtype=np.float64, buffer=self._shm_buffer, offset=self._offset_joint_target)
        self._cartesian_target_shm = np.ndarray((6,), dtype=np.float64, buffer=self._shm_buffer, offset=self._offset_cartesian_target)
        self._joint_state_shm = np.ndarray((6,), dtype=np.float64, buffer=self._shm_buffer, offset=self._offset_joint_state)
        self._cartesian_state_shm = np.ndarray((6,), dtype=np.float64, buffer=self._shm_buffer, offset=self._offset_cartesian_state)

        # Initialise with -10 to check
        self._joint_state_shm[:] = -10

        # Start the robot control process
        self._robot_process = mp.Process(
            target=_control_robot, 
            args=(SHM_NAME, self._ip, self._stop_queue, self._config_queue)
        )
        self._robot_process.daemon = True # Kills process if main process exits
        self._robot_process.start()

        # Check for first update
        while self._joint_state_shm[0]==-10:
            print("Waiting for first robot state to arrive..")
            time.sleep(1)
        print("Robot Connection established.")


    def __del__(self):
        """Ensures resources are cleaned up when the object is garbage collected."""
        self.stop_control_process()
    
    def stop_control_process(self):
        """Safely stops the robot control process."""
        if self._robot_process.is_alive():
            self._stop_queue.put("STOP")
            self._robot_process.join(timeout=5)
            if self._robot_process.is_alive():
                self._robot_process.terminate()
            self._shm.close()
            self._shm.unlink()
            print("UR5e control process stopped and shared memory unlinked.")

    def get_cartesian_position(self) -> common.Pose:
        ur_pose = self._cartesian_state_shm
        trans = ur_pose[0:3]
        rotvec = common.RotVec(np.array(ur_pose[3:6]))
        pose = common.Pose(quaternion=rotvec.as_quaternion_vector(), translation=trans)
        pose = common.Pose(rpy_vector=[0, 0, np.deg2rad(180)], translation=[0, 0, 0]).inverse() * pose
        return pose

    def get_ik(self) -> common.IK | None:

        return None

    def get_joint_position(self) -> np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]]:
        return np.array(self._joint_state_shm)

    def get_parameters(self) -> UR5eConfig:
        return self._config

    def set_parameters(self, robot_cfg: UR5eConfig) -> None:
        self._config = robot_cfg
        # self._config_queue.put(robot_cfg)

    def get_state(self) -> UR5eState:
        return UR5eState

    def set_cartesian_position(self, pose: common.Pose) -> None:
        q = self.ik.ik(pose, self.get_joint_position())
        if q is None:
            print("IK failed")
            return
        self.set_joint_position(q)
        return
        self._shm_buffer[self._offset_target_reached] = 0
        target_pose = (common.Pose(rpy_vector=[0, 0, np.deg2rad(180)], translation=[0, 0, 0]) * pose).rotvec()
        self._cartesian_target_shm[:] = target_pose
        self._shm_buffer[self._offset_mode:self._offset_target_reached] = (CARTESIAN_MODE).to_bytes(4, "little")
        if not self._config.async_control:
            while not self._shm_buffer[self._offset_target_reached]:
                time.sleep(0.01)

    def set_joint_position(self, q: np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]]) -> None:
        self._shm_buffer[self._offset_target_reached] = 0
        self._joint_target_shm[:] = q
        self._shm_buffer[self._offset_mode:self._offset_target_reached] = (JOINT_MODE).to_bytes(4, "little")
        if not self._config.async_control:
            while not self._shm_buffer[self._offset_target_reached]:
                time.sleep(0.01)
    
    def move_home(self) -> None:
        home = typing.cast(
            np.ndarray[tuple[typing.Literal[6]], np.dtype[np.float64]],
            common.robots_meta_config(common.RobotType.UR5e).q_home,
        )
        if np.any((home < -2 * np.pi) | (home > 2 * np.pi)):
            raise ValueError(f"Home position {home} is out of bounds.")
        print(f"Moving to home position: {home}")
        self._joint_target_shm[:] = home
        self._shm_buffer[self._offset_mode:self._offset_target_reached] = (JOINT_MODE).to_bytes(4, "little")
        while not self._shm_buffer[self._offset_target_reached]:
            time.sleep(0.01)

    def reset(self) -> None:
        pass

class RobotiQGripper:  # (common.Gripper):
    def __init__(self, ip):
        self.gripper = robotiq_gripper.RobotiqGripper()
        try:
            self.gripper.connect(ip, 63352, socket_timeout=3.0)  # default port for Robotiq gripper
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Robotiq gripper at {ip}: {e}")
        if not self.gripper.is_active():
            self.gripper.activate()
        print("Gripper Connection established.")

    def get_normalized_width(self) -> float:
        # value between 0 and 1 (0 is closed)
        return (self.gripper.get_max_position() - self.gripper.get_current_position()) / self.gripper.get_max_position()

    def grasp(self) -> None:
        """
        Close the gripper to grasp an object.
        """
        self.set_normalized_width(0.0)

    # def is_grasped(self) -> bool: ...

    def open(self) -> None:
        """
        Open the gripper to its maximum width.
        """
        self.set_normalized_width(1.0)

    def reset(self) -> None:
        self.open()

    def set_normalized_width(self, width: float, _: float = 0) -> None:
        """
        Set the gripper width to a normalized value between 0 and 1.
        """
        if not (0 <= width <= 1):
            msg = f"Width must be between 0 and 1, got {width}."
            raise ValueError(msg)
        abs_width = (1 - width) * self.gripper.get_max_position()
        # print(f"Setting gripper width to {width:.2f} (absolute: {abs_width:.2f})")
        self.gripper.move(int(abs_width), int(self.gripper._max_speed), int(self.gripper._max_force))

    def shut(self) -> None:
        """
        Close the gripper.
        """
        self.set_normalized_width(0.0)
