import numpy as np
import pytest
from rcs._core.sim import SimConfig
from rcs.envs.configs import EmptyWorldFR3, EmptyWorldFR3Duo
from rcs.envs.scenes import SimEnvCreatorConfig

FREQUENCY = 15


def _cfg(creator, record: bool, async_control: bool = True) -> SimEnvCreatorConfig:
    cfg = creator.config()
    cfg.sim_cfg = SimConfig(async_control=async_control, realtime=False, frequency=FREQUENCY, max_convergence_steps=500)
    cfg.headless = True
    cfg.camera_cfgs = None
    cfg.camera_adds = None
    cfg.wrapper_cfg.record_control_states = record
    return cfg


def _zero_action(env):
    return {
        name: {"tquat": np.array([0, 0, 0, 0, 0, 0, 1.0]), "gripper": np.array([1.0])}
        for name in env.get_wrapper_attr("envs")
    }


@pytest.mark.parametrize("creator_cls", [EmptyWorldFR3, EmptyWorldFR3Duo])
def test_sim_control_states_match_hardware_layout(creator_cls):
    creator = creator_cls()
    env = creator.create_env(_cfg(creator, record=True))
    try:
        env.reset()
        sim = env.get_wrapper_attr("sim")
        expected = round(1 / FREQUENCY / sim.model.opt.timestep)
        for _ in range(3):
            _, _, _, _, info = env.step(_zero_action(env))
        for robot_name in env.get_wrapper_attr("envs"):
            states = info[robot_name]["control_states"]
            n = states["num_samples"]
            assert n == expected
            assert states["timestamp"].shape == (n,)
            assert states["q"].shape == (n, 7)
            assert states["dq"].shape == (n, 7)
            assert states["tau_J"].shape == (n, 7)
            assert states["dtau_J"].shape == (n, 7)
            assert states["tau_ext_hat_filtered"].shape == (n, 7)
            assert states["O_F_ext_hat_K"].shape == (n, 6)
            assert states["O_T_EE"].shape == (n, 16)
            assert states["robot_mode"].shape == (n,)
            # robot time advances by one physics step per sample
            assert np.allclose(np.diff(states["robot_time"]), sim.model.opt.timestep)
            # the last sample is the state the observation was taken in
            o_t_ee = states["O_T_EE"][-1].reshape(4, 4, order="F")
            assert np.allclose(o_t_ee[3], [0, 0, 0, 1])
    finally:
        env.close()


def test_sim_control_states_are_opt_in():
    creator = EmptyWorldFR3()
    env = creator.create_env(_cfg(creator, record=False))
    try:
        env.reset()
        _, _, _, _, info = env.step(_zero_action(env))
        assert "control_states" not in info[next(iter(env.get_wrapper_attr("envs")))]
        assert not getattr(env.get_wrapper_attr("sim"), "substep_callbacks", [])
    finally:
        env.close()


def test_sim_control_states_sync_mode_records_one_sample():
    creator = EmptyWorldFR3()
    env = creator.create_env(_cfg(creator, record=True, async_control=False))
    try:
        env.reset()
        _, _, _, _, info = env.step(_zero_action(env))
        assert info[next(iter(env.get_wrapper_attr("envs")))]["control_states"]["num_samples"] == 1
    finally:
        env.close()
