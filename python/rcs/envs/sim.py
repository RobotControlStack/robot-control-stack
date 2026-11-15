import logging
import time
from typing import Any, SupportsFloat, cast

import gymnasium as gym
import mujoco as mj
import numpy as np
from rcs._core.common import RobotPlatform
from rcs.envs.space_utils import ActObsInfoWrapper

from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class RobotSimWrapper(ActObsInfoWrapper):
    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.SIMULATION, "Base environment must be simulation."
        assert isinstance(self.get_wrapper_attr("robot"), sim.SimRobot), "Robot must be a sim.SimRobot instance."
        self.sim_robot = cast(sim.SimRobot, self.get_wrapper_attr("robot"))
        self.sim = cast(sim.Sim, self.get_wrapper_attr("sim"))

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        self.sim_robot.clear_collision_flag()
        return action

    def observation(self, observation: dict, info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        state = self.sim_robot.get_state()
        if "collision" not in info:
            info["collision"] = state.collision
        else:
            info["collision"] = info["collision"] or state.collision
        info["ik_success"] = state.ik_success
        info["is_sim_converged"] = self.sim.is_converged()
        return observation, info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self.sim_robot.clear_collision_flag()
        return super().reset(seed=seed, options=options)


class SimControlStateWrapper(gym.Wrapper):
    """Simulation counterpart of the control loop state recording of the hardware (see ``FR3HW``).

    Samples the robot state after every physics step and adds all samples since the last env step to
    ``info["control_states"]`` with the same keys and shapes as on the hardware, so that recordings, tests
    and the data viewer work the same in simulation. Only ``async_control`` steps the physics in fixed
    sub steps, with ``step_until_convergence`` one sample per env step is recorded.

    Notes on the simulated signals: ``tau_J`` is the actuator torque in joint space, ``tau_J_d`` the
    actuator force, ``dtau_J`` the finite difference of ``tau_J``, ``tau_ext_hat_filtered`` the constraint
    (contact and limit) torques, ``O_F_ext_hat_K`` the wrench at the attachment site that explains these
    torques (least squares through the jacobian) in the robot base frame, ``K_F_ext_hat_K`` the same
    wrench in the attachment site frame and ``O_T_EE`` the column major pose of the attachment site in
    the robot base frame. ``robot_mode``, ``current_errors`` and ``last_motion_errors`` are always zero.
    """

    CONTROL_STATES_KEY = "control_states"

    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.SIMULATION, "Base environment must be simulation."
        self.sim = cast(sim.Sim, self.get_wrapper_attr("sim"))
        cfg = cast(sim.SimRobotConfig, self.get_wrapper_attr("robot").get_config())
        m = self.sim.model
        joint_ids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, name) for name in cfg.joints]
        actuator_ids = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_ACTUATOR, name) for name in cfg.actuators]
        assert all(i >= 0 for i in joint_ids), f"joints {cfg.joints} not found in the model"
        self._qpos_adr = m.jnt_qposadr[joint_ids]
        self._dof_adr = m.jnt_dofadr[joint_ids]
        self._actuator_ids = np.array([i for i in actuator_ids if i >= 0], dtype=int)
        self._site_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, cfg.attachment_site)
        self._base_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, cfg.base)
        self._jacp = np.zeros((3, m.nv))
        self._jacr = np.zeros((3, m.nv))
        self._samples: list[dict[str, Any]] = []
        self._prev_tau: np.ndarray | None = None
        if not hasattr(self.sim, "substep_callbacks"):
            self.sim.substep_callbacks = []  # type: ignore[attr-defined]
        self.sim.substep_callbacks.append(self._sample)  # type: ignore[attr-defined]

    def _base_pose(self) -> tuple[np.ndarray, np.ndarray]:
        d = self.sim.data
        if self._base_id < 0:
            return np.zeros(3), np.eye(3)
        return d.xpos[self._base_id], d.xmat[self._base_id].reshape(3, 3)

    def _sample(self):
        m, d = self.sim.model, self.sim.data
        tau = d.qfrc_actuator[self._dof_adr].copy()
        dt = m.opt.timestep
        dtau = np.zeros_like(tau) if self._prev_tau is None else (tau - self._prev_tau) / dt
        self._prev_tau = tau
        tau_ext = d.qfrc_constraint[self._dof_adr] + d.qfrc_applied[self._dof_adr]

        base_pos, base_rot = self._base_pose()
        o_t_ee = np.eye(4)
        o_f_ext = np.zeros(6)
        k_f_ext = np.zeros(6)
        if self._site_id >= 0:
            site_rot_o = base_rot.T @ d.site_xmat[self._site_id].reshape(3, 3)
            o_t_ee[:3, :3] = site_rot_o
            o_t_ee[:3, 3] = base_rot.T @ (d.site_xpos[self._site_id] - base_pos)
            mj.mj_jacSite(m, d, self._jacp, self._jacr, self._site_id)
            jac = np.vstack([self._jacp[:, self._dof_adr], self._jacr[:, self._dof_adr]])
            # tau_ext = J^T F (world frame) -> least squares wrench, then into the base frame
            f_world = np.linalg.lstsq(jac.T, tau_ext, rcond=None)[0]
            o_f_ext = np.concatenate([base_rot.T @ f_world[:3], base_rot.T @ f_world[3:]])
            k_f_ext = np.concatenate([site_rot_o.T @ o_f_ext[:3], site_rot_o.T @ o_f_ext[3:]])

        self._samples.append(
            {
                "timestamp": time.time(),
                "robot_time": d.time,
                "q": d.qpos[self._qpos_adr].copy(),
                "dq": d.qvel[self._dof_adr].copy(),
                "tau_J": tau,
                "tau_J_d": d.actuator_force[self._actuator_ids].copy(),
                "dtau_J": dtau,
                "tau_ext_hat_filtered": tau_ext,
                "O_F_ext_hat_K": o_f_ext,
                "K_F_ext_hat_K": k_f_ext,
                "O_T_EE": o_t_ee.flatten(order="F"),
            }
        )

    def drain_control_states(self) -> dict[str, Any]:
        n = len(self._samples)
        out: dict[str, Any] = {"num_samples": n, "dropped": 0}
        widths = {"q": 0, "dq": 0, "tau_J": 0, "tau_J_d": 0, "dtau_J": 0, "tau_ext_hat_filtered": 0}
        for key in ("timestamp", "robot_time"):
            out[key] = np.array([s[key] for s in self._samples], dtype=np.float64)
        for key, width in [*widths.items(), ("O_F_ext_hat_K", 6), ("K_F_ext_hat_K", 6), ("O_T_EE", 16)]:
            dim = width or len(self._dof_adr)
            out[key] = (
                np.stack([s[key] for s in self._samples]).astype(np.float64) if n else np.zeros((0, dim), np.float64)
            )
        out["control_command_success_rate"] = np.ones(n, dtype=np.float64)
        out["robot_mode"] = np.zeros(n, dtype=np.int32)
        out["current_errors"] = np.zeros(n, dtype=np.uint64)
        out["last_motion_errors"] = np.zeros(n, dtype=np.uint64)
        self._samples = []
        return out

    def step(self, action: Any) -> tuple[dict[str, Any], SupportsFloat, bool, bool, dict[str, Any]]:
        self._samples = []
        obs, reward, terminated, truncated, info = super().step(action)
        if not self._samples:
            # no fixed sub steps (step_until_convergence), record the state after the step
            self._sample()
        info[self.CONTROL_STATES_KEY] = self.drain_control_states()
        return obs, reward, terminated, truncated, info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs, info = super().reset(seed=seed, options=options)
        self._samples = []
        self._prev_tau = None
        return obs, info

    def close(self):
        callbacks = getattr(self.sim, "substep_callbacks", [])
        if self._sample in callbacks:
            callbacks.remove(self._sample)
        super().close()


class GripperWrapperSim(ActObsInfoWrapper):
    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.SIMULATION, "Base environment must be simulation."
        assert isinstance(
            self.get_wrapper_attr("gripper"), sim.SimGripper
        ), "Gripper must be a sim.SimGripper instance."
        self._gripper = cast(sim.SimGripper, self.get_wrapper_attr("gripper"))

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        self._gripper.clear_collision_flag()
        return action

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        state = self._gripper.get_state()
        if "collision" not in info or not info["collision"]:
            info["collision"] = state.collision
        info["gripper_width"] = self._gripper.get_normalized_width()
        info["is_grasped"] = self._gripper.get_normalized_width() > 0.01 and self._gripper.get_normalized_width() < 0.99
        return observation, info

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self._gripper.clear_collision_flag()
        return super().reset(seed=seed, options=options)


class HandWrapperSim(ActObsInfoWrapper):
    def __init__(self, env):
        super().__init__(env)
        assert self.env.get_wrapper_attr("PLATFORM") == RobotPlatform.SIMULATION, "Base environment must be simulation."
        assert isinstance(
            self.get_wrapper_attr("hand"), sim.SimTilburgHand
        ), "Hand must be a sim.SimTilburgHand instance."
        self._hand = cast(sim.SimTilburgHand, self.get_wrapper_attr("hand"))

    def action(self, action: dict[str, Any]) -> dict[str, Any]:
        if isinstance(action["hand"], int | float):
            return action
        if len(action["hand"]) == 18:
            action["hand"] = action["hand"][:16]
        assert len(action["hand"]) == 16 or len(action["hand"]) == 1, "Hand action must be of length 16 or 1"
        return action

    def observation(self, observation: dict[str, Any], info: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        state = self._hand.get_state()
        if "collision" not in info or not info["collision"]:
            info["collision"] = state.collision
        info["hand_position"] = self._hand.get_normalized_joint_poses()
        # info["is_grasped"] = self._hand.get_normalized_joint_poses() > 0.01 and self._hand.get_normalized_joint_poses() < 0.99
        return observation, info


class DigitalTwin(gym.Wrapper):
    def __init__(self, env, twin_env):
        super().__init__(env)
        self.twin_env = twin_env

    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)

        twin_obs, _, _, _, _ = self.twin_env.step(obs)
        info["twin_obs"] = twin_obs
        return obs, reward, terminated, truncated, info
