from dataclasses import dataclass

from rcs_fr3._core import hw as fr3_hw
from rcs_fr3.configs import DefaultFR3HardwareEnv
from rcs_fr3_bilateral._core import hw
import numpy as np

@dataclass
class DefaultFR3BilateralTeleop:
    leader_ip: str = "192.168.101.1"
    follower_ip: str = "192.168.102.1"
    gravity_only: bool = False
    ignore_realtime: bool = False
    update_rate_hz: float = 1000.0
    relative_joint_mapping: bool = True
    max_follower_joint_step: float = 0.05
    haptic_feedback_gain: float = 1.5
    leader_haptic_feedback: bool = True

    def _robot_config(self, ip: str) -> fr3_hw.FR3Config:
        base = DefaultFR3HardwareEnv()
        base.ip = ip
        robot_cfg = base.config().robot_cfg
        robot_cfg.async_control = True
        robot_cfg.ignore_realtime = self.ignore_realtime
        # original KP: np.array([100., 100., 100., 100., 75., 150., 50.]) 
        # Tuned values optimized for haptics
        robot_cfg.joint_controller_Kp = np.array([100., 100., 100., 100., 75., 75., 30.])
        robot_cfg.joint_controller_Kp = 2 * robot_cfg.joint_controller_Kp
        robot_cfg.joint_controller_Kd = 2*np.sqrt(robot_cfg.joint_controller_Kp)
        
        return robot_cfg

    def config(self) -> hw.BilateralFrankaConfig:
        return hw.BilateralFrankaConfig(
            leader_cfg=self._robot_config(self.leader_ip),
            follower_cfg=self._robot_config(self.follower_ip),
            control_mode=(
                hw.BilateralControlMode.gravity_only if self.gravity_only else hw.BilateralControlMode.bilateral
            ),
            update_rate_hz=self.update_rate_hz,
            relative_joint_mapping=self.relative_joint_mapping,
            max_follower_joint_step=self.max_follower_joint_step,
            haptic_feedback_gain=self.haptic_feedback_gain,
            leader_haptic_feedback=self.leader_haptic_feedback,
        )
