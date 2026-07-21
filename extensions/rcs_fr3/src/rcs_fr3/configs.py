import copy

import numpy as np
from rcs.envs.base import ControlMode, RelativeTo
from rcs_fr3._core import hw
from rcs_fr3.creators import (
    FR3HardwareEnvCreatorConfig,
    FR3MultiHardwareEnvCreatorConfig,
    HardwareCameraCreatorConfig,
    RCSFR3ConfigEnvCreator,
    RCSFR3MultiConfigEnvCreator,
)

import rcs
from rcs import common


class DefaultFR3HardwareEnv(RCSFR3ConfigEnvCreator):
    ip = "192.168.101.1"

    def config(self) -> FR3HardwareEnvCreatorConfig:
        fr3 = rcs.ROBOTS[common.RobotType.FR3]
        robot_cfg = hw.FR3Config(
            ip=self.ip,
            ik_solver=hw.IKSolver.rcs_ik,
            speed_factor=0.1,
            async_control=False,
            tcp_offset_configured_in_desk=True,
            ignore_realtime=False,
            tcp_offset=common.Pose(common.FrankaHandTCPOffset()),
            attachment_site="attachment_site",
            kinematic_model_path=fr3.mjcf_model_path,
        )
        robot_cfg.robot_type = common.RobotType.FR3
        robot_cfg.q_home = fr3.q_home
        robot_cfg.joint_limits = fr3.joint_limits
        robot_cfg.dof = fr3.dof

        gripper_cfg = hw.FHConfig(
            ip=self.ip,
            grasping_width=0.05,
            speed=0.1,
            force=30.0,
            epsilon_inner=0.1,
            epsilon_outer=0.1,
            async_control=False,
        )

        return FR3HardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TRPY,
            robot_cfg=robot_cfg,
            gripper_cfg=gripper_cfg,
            camera_cfgs={
                "realsense": HardwareCameraCreatorConfig(
                    camera_type_id="realsense",
                    camera_cfgs={
                        "left_wrist": common.BaseCameraConfig(
                            identifier="230422272017",
                            resolution_width=1280,
                            resolution_height=720,
                            frame_rate=30,
                        ),
                        "right_wrist": common.BaseCameraConfig(
                            identifier="230422271040",
                            resolution_width=1280,
                            resolution_height=720,
                            frame_rate=30,
                        ),
                        "side": common.BaseCameraConfig(
                            identifier="243522070385",
                            resolution_width=1280,
                            resolution_height=720,
                            frame_rate=30,
                        ),
                        "bird_eye": common.BaseCameraConfig(
                            identifier="243522070364",
                            resolution_width=1280,
                            resolution_height=720,
                            frame_rate=30,
                        ),
                    },
                ),
                "digit": HardwareCameraCreatorConfig(
                    camera_type_id="digit",
                    camera_cfgs={
                        "digit_right_left": common.BaseCameraConfig(
                            identifier="D21182",
                            resolution_width=320,
                            resolution_height=240,
                            frame_rate=30,
                        ),
                        "digit_right_right": common.BaseCameraConfig(
                            identifier="D21193",
                            resolution_width=320,
                            resolution_height=240,
                            frame_rate=30,
                        ),
                    },
                ),
            },
            max_relative_movement=(0.2, np.deg2rad(45)),
            relative_to=RelativeTo.LAST_STEP,
        )


class DROIDEnv(RCSFR3MultiConfigEnvCreator):
    robot_ip = "192.168.101.1"
    gripper_serial_number = "DAAQMPDC"

    def config(self) -> FR3MultiHardwareEnvCreatorConfig:
        try:
            from rcs_robotiq2f85.hw import RobotiQ2F85GripperConfig
        except ImportError as e:
            msg = "Robotiq gripper support requires the `rcs_robotiq2f85` extension to be installed."
            raise ImportError(msg) from e

        base = DefaultFR3HardwareEnv()

        cfg = base.config()
        cfg.robot_cfg.async_control = True
        cfg.robot_cfg.ip = self.robot_ip
        cfg.robot_cfg.tcp_offset = rcs.GRIPPER_TCP_OFFSETS[common.GripperType("Robotiq2F85")]
        cfg.robot_cfg.q_home = rcs.HOME_POSITIONS["FR3_DROID"]

        return FR3MultiHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TRPY,
            robot_cfgs={
                "right": cfg.robot_cfg,
            },
            camera_cfgs=None,
            max_relative_movement=(0.5, np.deg2rad(90)),
            relative_to=RelativeTo.LAST_STEP,
            gripper_cfgs={
                "right": RobotiQ2F85GripperConfig(
                    serial_number=self.gripper_serial_number,
                    speed=100,
                    force=50,
                    async_control=True,
                ),
            },
            robot_to_shared_base_frame={
                "right": common.Pose(
                    translation=np.array([0, 0, 0]),
                    rpy_vector=np.array([0, 0, 0]),
                ),
            },
        )


class DefaultFR3DualMultiHardwareEnv(RCSFR3MultiConfigEnvCreator):
    left_ip = "192.168.101.1"
    right_ip = "192.168.102.1"

    def config(self) -> FR3MultiHardwareEnvCreatorConfig:
        base = DefaultFR3HardwareEnv()

        base.ip = self.left_ip
        left_cfg = base.config()
        left_cfg.robot_cfg.async_control = True
        if isinstance(left_cfg.gripper_cfg, hw.FHConfig):
            left_cfg.gripper_cfg.async_control = True

        base.ip = self.right_ip
        right_cfg = base.config()
        right_cfg.robot_cfg.async_control = True
        if isinstance(right_cfg.gripper_cfg, hw.FHConfig):
            right_cfg.gripper_cfg.async_control = True

        return FR3MultiHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TRPY,
            robot_cfgs={
                "left": left_cfg.robot_cfg,
                "right": right_cfg.robot_cfg,
            },
            gripper_cfgs={
                "left": left_cfg.gripper_cfg,
                "right": right_cfg.gripper_cfg,
            },
            camera_cfgs=copy.deepcopy(left_cfg.camera_cfgs),
            max_relative_movement=(0.5, np.deg2rad(90)),
            relative_to=RelativeTo.LAST_STEP,
            # this is an example how the robots are oriented to each other
            # in this example the have an aloha like mounting, facing each other
            # with a distance of 1.5m
            robot_to_shared_base_frame={
                "right": common.Pose(
                    translation=np.array([0, 0, 0]),
                    rpy_vector=np.array([0, 0, 0]),
                ),
                "left": common.Pose(
                    translation=np.array([0, 0, 0]),
                    rpy_vector=np.array([0, 0, np.pi]),
                ),
            },
        )


class FrankaDuoEnv(DefaultFR3DualMultiHardwareEnv):
    # use `udevadm info -a -n /dev/ttyUSB0 | grep serial` to find out the serial numbers
    left_gripper_serial_number = "DAAQMJHX"
    right_gripper_serial_number = "DAAQMPDC"

    def config(self) -> FR3MultiHardwareEnvCreatorConfig:
        try:
            from rcs_robotiq2f85.hw import RobotiQ2F85GripperConfig
        except ImportError as e:
            msg = "Robotiq gripper support requires the `rcs_robotiq2f85` extension to be installed."
            raise ImportError(msg) from e

        cfg = super().config()
        cfg.camera_cfgs = None
        cfg.robot_cfgs["left"].tcp_offset = rcs.GRIPPER_TCP_OFFSETS[common.GripperType("Robotiq2F85")]
        cfg.robot_cfgs["right"].tcp_offset = rcs.GRIPPER_TCP_OFFSETS[common.GripperType("Robotiq2F85")]
        cfg.robot_cfgs["left"].q_home = rcs.HOME_POSITIONS["FR3_DUO_LEFT"]
        cfg.robot_cfgs["right"].q_home = rcs.HOME_POSITIONS["FR3_DUO_RIGHT"]
        cfg.gripper_cfgs = {
            "left": RobotiQ2F85GripperConfig(
                serial_number=self.left_gripper_serial_number,
                speed=100,
                force=50,
                async_control=True,
            ),
            "right": RobotiQ2F85GripperConfig(
                serial_number=self.right_gripper_serial_number,
                speed=100,
                force=50,
                async_control=True,
            ),
        }
        cfg.robot_to_shared_base_frame = {
            "left": copy.deepcopy(rcs.DEFAULT_TRANSFORMS["FR3_DUOMOUNT_LEFT_ROBOT"]),
            "right": copy.deepcopy(rcs.DEFAULT_TRANSFORMS["FR3_DUOMOUNT_RIGHT_ROBOT"]),
        }
        return cfg

class SingleArmFR3MultiHardwareEnv(RCSFR3MultiConfigEnvCreator):
    
    gripper_serial_number = "DAAQMJHX" # "DAAQMPDC"

    def config(self, grippertype=common.GripperType.FrankaHand, robot_ip="192.168.101.1") -> FR3MultiHardwareEnvCreatorConfig:
        self.robot_ip = robot_ip
        base = DefaultFR3HardwareEnv()
        base.ip = self.robot_ip
        right_cfg = base.config()
        right_cfg.robot_cfg.async_control = True
        right_cfg.robot_cfg.ignore_realtime = True
        if isinstance(right_cfg.gripper_cfg, hw.FHConfig):
            right_cfg.gripper_cfg.async_control = True
        right_cfg.robot_cfg.q_home = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi/4])
        right_cfg.robot_cfg.tcp_offset = rcs.GRIPPER_TCP_OFFSETS[grippertype]

        if grippertype == common.GripperType("Robotiq2F85"):
            try:
                from rcs_robotiq2f85.hw import RobotiQ2F85GripperConfig
            except ImportError as e:
                msg = "Robotiq gripper support requires the `rcs_robotiq2f85` extension to be installed."
                raise ImportError(msg) from e
            right_cfg.robot_cfg.q_home = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, 0])
            right_cfg.gripper_cfg = RobotiQ2F85GripperConfig(
                serial_number=self.gripper_serial_number,
                speed=100,
                force=50,
                async_control=True,
            )
        return FR3MultiHardwareEnvCreatorConfig(
            control_mode=ControlMode.CARTESIAN_TRPY,
            robot_cfgs={
                "right": right_cfg.robot_cfg,
            },
            gripper_cfgs={
                "right": right_cfg.gripper_cfg,
            },
            camera_cfgs=copy.deepcopy(right_cfg.camera_cfgs),
            max_relative_movement=(0.5, np.deg2rad(90)),
            relative_to=RelativeTo.LAST_STEP,
            # this is an example how the robots are oriented to each other
            # in this example the have an aloha like mounting, facing each other
            # with a distance of 1.5m
            robot_to_shared_base_frame={
                "right": common.Pose(
                    translation=np.array([0, 0, 0]),
                    rpy_vector=np.array([0, 0, 0]),
                ),
            },
        )
