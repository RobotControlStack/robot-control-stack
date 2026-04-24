import copy
import logging
import threading
import time
from abc import ABC
from dataclasses import dataclass, field
from time import sleep

import gymnasium as gym
from rcs.envs.base import ArmWithGripper, ControlMode, RelativeTo
from rcs.sim.sim import Sim
from rcs.utils import SimpleFrameRate

logger = logging.getLogger(__name__)


@dataclass
class TeleopCommands:
    """Semantic commands decoupled from specific hardware buttons."""

    record: bool = False
    success: bool = False
    failure: bool = False
    sync_position: bool = False
    reset_origin_to_current: dict[str, bool] = field(default_factory=dict)


class BaseOperator(ABC, threading.Thread):
    control_mode: tuple[ControlMode, RelativeTo]
    controller_names: list[str]

    def __init__(self, config: "BaseOperatorConfig", sim: Sim | None = None):
        threading.Thread.__init__(self)
        self.config = config
        self.sim = sim

    def consume_commands(self) -> TeleopCommands:
        """Returns the current commands and resets them (edge-triggered). Must be thread-safe."""
        raise NotImplementedError()

    def reset_operator_state(self):
        """Hook for subclasses to reset their internal poses/offsets on env reset. Must be thread-safe."""

    def run(self):
        """Read out hardware, set states and process buttons."""
        raise NotImplementedError()

    def consume_action(self) -> dict[str, ArmWithGripper]:
        """Returns the action dictionary to step the environment. Must be thread-safe."""
        raise NotImplementedError()

    def close(self):
        pass


@dataclass(kw_only=True)
class BaseOperatorConfig:
    operator_class: type[BaseOperator]
    read_frequency: int = 30
    simulation: bool = True


class TeleopLoop:
    """Interface for an operator device"""

    # Define this as a class attribute so it can be accessed without instantiating
    control_mode: tuple[ControlMode, RelativeTo]

    def __init__(
        self,
        env: gym.Env,
        operator: BaseOperator,
        env_frequency: int = 30,
        key_translation: dict[str, str] | None = None,
    ):
        super().__init__()
        self.env = env
        self.operator = operator
        self._exit_requested = False
        self.env_frequency = env_frequency
        if key_translation is None:
            # controller to robot translation
            self.key_translation = {key: key for key in self.operator.controller_names}
        else:
            self.key_translation = key_translation

        # Absolute operators (RelativeTo.NONE) need an initial sync
        self._synced = self.operator.control_mode[1] != RelativeTo.NONE

    def stop(self):
        self.operator.close()
        self._exit_requested = True
        self.operator.join()

    def __enter__(self):
        self.operator.start()
        # sleep(2)
        return self

    def __exit__(self, *_):
        self.stop()

    def _translate_keys(self, actions):
        translated = {self.key_translation[key]: actions[key] for key in actions}
        # Fill in missing robots with "hold" actions from last observation
        # This is necessary because absolute environments (like MultiRobotWrapper)
        # require actions for all configured robots in every step.
        for robot_name in self.env.get_wrapper_attr("envs"):
            if robot_name not in translated and robot_name in self._last_obs:
                translated[robot_name] = {
                    "joints": self._last_obs[robot_name]["joints"].copy(),
                    "gripper": self._last_obs[robot_name].get("gripper", 1.0),
                }
        return translated

    def environment_step_loop(self):
        rate_limiter = SimpleFrameRate(self.env_frequency, "env loop")

        # 0. Initial Reset to get current positions for untracked robots
        self._last_obs, _ = self.env.reset()

        while True:
            if self._exit_requested:
                break

            # 1. Process Meta-Commands
            cmds = self.operator.consume_commands()

            if cmds.record:
                print("Command: Start Recording")
                self.env.get_wrapper_attr("start_record")()

            if cmds.success:
                print("Command: Success! Resetting env...")
                self.env.get_wrapper_attr("success")()
                sleep(1)  # sleep to let the robot reach the goal
                self._last_obs, _ = self.env.reset()
                self.operator.reset_operator_state()
                self._synced = self.operator.control_mode[1] != RelativeTo.NONE
                # consume new commands because of potential origin reset
                continue

            if cmds.failure:
                print("Command: Failure! Resetting env...")
                self._last_obs, _ = self.env.reset()
                self.operator.reset_operator_state()
                self._synced = self.operator.control_mode[1] != RelativeTo.NONE
                # consume new commands because of potential origin reset
                continue

            if cmds.sync_position:
                self.sync_robot_to_operator()
                self._synced = True
                continue

            if not self._synced:
                # Still waiting for sync, step the env with "hold" actions
                if int(time.time()) % 5 == 0 and int(time.time() * self.env_frequency) % self.env_frequency == 0:
                    print("Waiting for sync... (Press 's' on GELLO/Keyboard to sync)")

                hold_actions = {}
                for robot_name in self.env.get_wrapper_attr("envs"):
                    if robot_name in self._last_obs and "joints" in self._last_obs[robot_name]:
                        hold_actions[robot_name] = {
                            "joints": self._last_obs[robot_name]["joints"].copy(),
                            "gripper": self._last_obs[robot_name].get("gripper", 1.0),
                        }

                self._last_obs, _, _, _, _ = self.env.step(hold_actions)
                rate_limiter()
                continue

            for controller in cmds.reset_origin_to_current:
                if cmds.reset_origin_to_current[controller]:
                    robot = self.key_translation[controller]
                    print(f"Command: Resetting origin for {robot}...")
                    assert (
                        self.operator.control_mode[1]
                        == RelativeTo.CONFIGURED_ORIGIN
                        # TODO the following is a dict and can thus not easily be used like this
                        # and self.env.get_wrapper_attr("relative_to") == RelativeTo.CONFIGURED_ORIGIN
                    ), "both robot env and operator must be configured to relative_to.CONFIGURED_ORIGIN"
                    self.env.get_wrapper_attr("envs")[robot].get_wrapper_attr("set_origin_to_current")()

            # 2. Step the Environment
            actions = self.operator.consume_action()
            actions = self._translate_keys(actions)
            self._last_obs, _, _, _, _ = self.env.step(actions)

            rate_limiter()

    def sync_robot_to_operator(self, duration: float = 3.0):
        print(f"Command: Syncing robot to operator (duration: {duration}s)...")
        rate_limiter = SimpleFrameRate(self.env_frequency, "sync loop")
        num_steps = int(duration * self.env_frequency)

        # 1. Capture the initial state for interpolation
        start_obs = copy.deepcopy(self._last_obs)

        # 2. Interpolation Loop
        for i in range(num_steps):
            alpha = (i + 1) / num_steps
            # Re-consume operator action to follow moving target!
            target_actions = self._translate_keys(self.operator.consume_action())

            interp_actions = {}
            for robot_name, target in target_actions.items():
                try:
                    # Interpolate from FIXED start towards MOVING target
                    s_joints = start_obs[robot_name]["joints"]
                    t_joints = target["joints"]
                    interp_joints = s_joints + alpha * (t_joints - s_joints)

                    s_gripper = start_obs[robot_name].get("gripper", 1.0)
                    t_gripper = target.get("gripper", 1.0)
                    interp_gripper = s_gripper + alpha * (t_gripper - s_gripper)

                    interp_actions[robot_name] = {
                        "joints": interp_joints,
                        "gripper": interp_gripper,
                    }
                except (KeyError, TypeError):
                    continue

            self._last_obs, _, _, _, _ = self.env.step(interp_actions)
            rate_limiter()

        print("Sync Complete.")
