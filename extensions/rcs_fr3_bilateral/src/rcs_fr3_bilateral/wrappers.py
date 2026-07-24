"""Gym wrappers which expose :class:`BilateralFranka` to RCS.

``BilateralFranka`` is deliberately different from the normal RCS hardware
layout: it owns *both* Frankas and its worker threads perform the 1 kHz
control loop.  Consequently, a normal :class:`rcs.envs.base.RobotWrapper`
must not be placed below this wrapper -- its low-rate ``set_joint_position``
calls would race the bilateral controller.

The wrapper presents the follower as the RCS robot and the leader as the RCS
action source.  At stack tick ``k`` it returns the latest follower state and
passes the leader state sampled at tick ``k - 1`` down the stack.  Sampling
the new leader state only after constructing that action provides an explicit
one-RCS-tick delay and avoids mixing a leader sample with an unrelated
follower observation.
"""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
from rcs.envs.base import ArmObsType, ControlMode, JointsDictType
from rcs.envs.space_utils import ActObsInfoWrapper, get_space, get_space_keys
from rcs_fr3._core import hw as fr3_hw

from rcs_fr3_bilateral._core import hw


class BilateralFR3Wrapper(ActObsInfoWrapper):
    """Make a high-rate bilateral FR3 controller look like an RCS robot layer.

    The wrapper is intended to replace ``RobotWrapper`` in a hardware stack::

        teleop = hw.BilateralFranka(bilateral_config)
        env = HardwareEnv()
        env = BilateralFR3Wrapper(env, teleop)
        env = GripperWrapper(env, follower_gripper, binary=False)
        env = MultiRobotWrapper({"right": env})

    ``step`` accepts the regular RCS action only to preserve the Gym wrapper
    interface.  The ``"tquat"``, ``"joints"``, and ``"xyzrpy"`` entries
    passed towards the wrapped environment are replaced with the leader state
    from the *previous* RCS tick.
    The bilateral controller itself remains solely responsible for follower
    commands at ``BilateralFrankaConfig.update_rate_hz`` (normally 1000 Hz).

    Args:
        env: The inner RCS environment.  It must not contain a ``RobotWrapper``
            for either Franka.
        teleop: Started and stopped by this wrapper unless ``manage_teleop``
            is false.
        manage_teleop: Start on reset and stop on close.  Set false when the
            caller owns that lifecycle (for example, an existing process has
            already called ``teleop.start()``).
    """

    LEADER_ACTION_KEY = "bilateral_leader_action"
    FOLLOWER_STATE_KEY = "bilateral_follower_state"

    def __init__(
        self,
        env: gym.Env,
        teleop: hw.BilateralFranka,
        *,
        manage_teleop: bool = True,
    ):
        super().__init__(env)
        self.teleop = teleop
        self.manage_teleop = manage_teleop
        self.robot = teleop.get_follower()
        if not isinstance(self.robot, fr3_hw.Franka):
            raise TypeError("BilateralFranka follower must be an rcs_fr3.hw.Franka.")

        low, high = self.robot.get_config().joint_limits
        # ``robot_state`` is intentionally observation-only.  GripperWrapper
        # adds the gripper field to both spaces in the next stack layer.
        arm_space = get_space(ArmObsType, params={"joint_limits": {"low": low, "high": high}})
        self.action_space = arm_space
        # The standard arm observation keys are retained so camera, storage,
        # and policy wrappers can be composed without bilateral special cases.
        self.observation_space = arm_space
        self.joints_key = get_space_keys(JointsDictType)[0]
        self._control_mode_overrides = [ControlMode.JOINTS]
        self._previous_leader_action: dict[str, np.ndarray] | None = None
        self._last_dispatched_leader_action: dict[str, np.ndarray] | None = None
        self._robot_state_keys: list[str] | None = None

    def get_unwrapped_control_mode(self, idx: int) -> ControlMode:
        return self._control_mode_overrides[idx]

    def get_base_control_mode(self) -> ControlMode:
        return self._control_mode_overrides[0]

    def get_control_mode(self) -> ControlMode:
        return self._control_mode_overrides[-1]

    def override_control_mode(self, control_mode: ControlMode) -> None:
        self._control_mode_overrides.append(control_mode)

    @staticmethod
    def _copy_vector(vector: Any) -> np.ndarray:
        """Detach a pybind Eigen vector before the next 1 kHz update."""
        return np.asarray(vector, dtype=np.float64).copy()

    def _get_leader_arm_action(self) -> dict[str, np.ndarray]:
        """Return the leader state in the stack action schema.

        This intentionally contains only ``tquat``, ``joints``, and
        ``xyzrpy``.  Low-level ``robot_state`` remains follower-observation
        data and is never inserted into an action.
        """
        state = self.teleop.get_state()
        leader_pose = self.teleop.get_leader().get_cartesian_position()
        return {
            self.joints_key: self._copy_vector(state.leader_q),
            "tquat": np.concatenate([leader_pose.translation(), leader_pose.rotation_q()]),
            "xyzrpy": leader_pose.xyzrpy(),
        }

    def get_leader_action(self) -> dict[str, np.ndarray]:
        """Return the flat leader action consumed by the outer RCS wrappers."""
        return self._get_leader_arm_action()

    @staticmethod
    def _copy_action(action: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        return {key: value.copy() for key, value in action.items()}

    def _robot_state_to_dict(self, state: fr3_hw.RobotState) -> dict[str, Any]:
        """Match ``FR3HW``'s complete recordable Franka state representation."""
        if self._robot_state_keys is None:
            self._robot_state_keys = [
                attr for attr in dir(state) if not attr.startswith("__") and not callable(getattr(state, attr))
            ]
            # ``robot_mode`` is a pybind enum and is intentionally excluded by
            # FR3HW as well because it is not dataset-serializable.
            self._robot_state_keys.remove("robot_mode")
        return {key: getattr(state, key) for key in self._robot_state_keys}

    def _get_follower_arm_obs(self) -> ArmObsType:
        """Return the latest follower state in the standard RCS arm schema."""
        state = self.teleop.get_state()
        # Joint values come from BilateralFranka's mutex-protected aggregate
        # state.  Cartesian values are queried from the same follower object
        # to preserve the established RCS ArmObsType contract.
        follower_pose = self.robot.get_cartesian_position()
        return ArmObsType(
            joints=self._copy_vector(state.follower_q),
            tquat=np.concatenate([follower_pose.translation(), follower_pose.rotation_q()]),
            xyzrpy=follower_pose.xyzrpy(),
        )

    def get_robot_obs(self) -> ArmObsType:
        """Return the follower observation consumed by the outer RCS wrappers."""
        return self._get_follower_arm_obs()

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Substitute the previous stack-tick leader sample for ``joints``.

        The current leader sample is cached for the next invocation.  No
        command is sent to ``self.robot`` here; doing so would interfere with
        the high-rate bilateral control threads.
        """
        if self._previous_leader_action is None:
            # The first reset/step has no preceding tick, so use the reset
            # sample as its well-defined initial action.
            self._previous_leader_action = self._get_leader_arm_action()

        # GripperWrapper has already consumed the gripper command at this
        # point. Replace only the arm fields with the delayed leader state.
        stack_action = self._copy_action(self._previous_leader_action)
        self._last_dispatched_leader_action = self._copy_action(stack_action)
        self._previous_leader_action = self._get_leader_arm_action()
        return stack_action

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        follower_observation = dict(observation)
        state = self.teleop.get_state()
        follower_observation.update(self._get_follower_arm_obs())
        follower_state = self.robot.get_state()
        follower_observation["robot_state"] = self._robot_state_to_dict(follower_state.robot_state)
        # Keep the bilateral-specific data available without changing the
        # standard policy observation schema.
        robot_info = dict(info)
        dispatched_leader_action = self._copy_action(
            self._last_dispatched_leader_action
            if self._last_dispatched_leader_action is not None
            else self._get_leader_arm_action()
        )
        robot_info[self.LEADER_ACTION_KEY] = dispatched_leader_action
        # Match the standard RCS recording schema so joint-space dataset
        # converters can consume bilateral recordings without special cases.
        robot_info["absolute_action"] = dispatched_leader_action[self.joints_key].copy()
        robot_info[self.FOLLOWER_STATE_KEY] = {
            "joints": self._copy_vector(state.follower_q),
            "joint_velocities": self._copy_vector(state.follower_dq),
            "target_joints": self._copy_vector(state.follower_target_q),
            "external_torques": self._copy_vector(state.follower_external_tau),
            "running": state.running,
            "has_reference": state.has_reference,
        }
        return follower_observation, robot_info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        if self.manage_teleop:
            self.teleop.reset()
            if not self.teleop.is_running():
                self.teleop.start()
        self._previous_leader_action = self._get_leader_arm_action()
        self._last_dispatched_leader_action = self._copy_action(self._previous_leader_action)
        return super().reset(seed=seed, options=options)

    def close(self) -> None:
        try:
            if self.manage_teleop and self.teleop.is_running():
                self.teleop.stop()
        finally:
            super().close()


# A longer spelling reads nicely at call sites and leaves the short name
# available for existing code.
BilateralTeleoperationWrapper = BilateralFR3Wrapper

__all__ = ["BilateralFR3Wrapper", "BilateralTeleoperationWrapper"]
