"""PinocchioDynamics must compute the same dynamics as MuJoCo for the same model."""

import mujoco
import numpy as np
import pytest

import rcs
from rcs import common

FLANGE = "attachment_site"
TCP_OFFSET = np.array([0.0, 0.0, 0.1493])
# the end effector configured on our FR3s
PAYLOAD = common.Payload(
    mass=0.9,
    com=np.array([0.0, 0.0, 0.057]),
    inertia=np.diag([0.002768, 0.003149, 0.000564]),
)


def fr3_xml() -> str:
    return str(rcs.ROBOTS[common.RobotType.FR3].mjcf_model_path)


def mujoco_model(payload: common.Payload | None = None) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(fr3_xml())
    if payload is not None:
        flange = spec.site(FLANGE)
        body = spec.body("fr3_link7").add_body(pos=flange.pos, quat=flange.quat)
        body.mass = payload.mass
        body.ipos = payload.com
        i = payload.inertia
        body.fullinertia = [i[0, 0], i[1, 1], i[2, 2], i[0, 1], i[0, 2], i[1, 2]]
        body.explicitinertial = True
    model = spec.compile()
    model.dof_armature[:] = 0.0  # PinocchioDynamics ignores armature, like franka::Model
    return model


def mujoco_state(model: mujoco.MjModel, q: np.ndarray, dq: np.ndarray, tcp_offset: np.ndarray):
    data = mujoco.MjData(model)
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    gravity = data.qfrc_bias.copy()
    data.qvel[:] = dq
    mujoco.mj_forward(model, data)
    coriolis = data.qfrc_bias - gravity
    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass)

    site = model.site(FLANGE).id
    rotation = data.site_xmat[site].reshape(3, 3)
    tcp = data.site_xpos[site] + rotation @ tcp_offset
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacp, jacr, tcp, model.site_bodyid[site])
    return mass, gravity, coriolis, np.vstack([jacp, jacr]), tcp, rotation


def random_states(model: mujoco.MjModel, n: int = 20):
    rng = np.random.default_rng(0)
    low, high = model.jnt_range[:7, 0], model.jnt_range[:7, 1]
    for _ in range(n):
        yield low + (high - low) * rng.random(7), rng.uniform(-2.0, 2.0, 7)


@pytest.mark.parametrize("payload", [None, PAYLOAD], ids=["no_payload", "payload"])
def test_matches_mujoco(payload):
    tcp = common.Pose(translation=TCP_OFFSET)
    dynamics = common.PinocchioDynamics(fr3_xml(), False, FLANGE, tcp)
    if payload is not None:
        dynamics.set_payload(payload)
    model = mujoco_model(payload)
    for q, dq in random_states(model):
        state = dynamics.compute(q, dq, 0.001)
        mass, gravity, coriolis, jacobian, tcp_pos, tcp_rot = mujoco_state(model, q, dq, TCP_OFFSET)
        np.testing.assert_allclose(state.mass, mass, atol=1e-8)
        np.testing.assert_allclose(state.gravity, gravity, atol=1e-8)
        np.testing.assert_allclose(state.coriolis, coriolis, atol=1e-8)
        np.testing.assert_allclose(state.jacobian, jacobian, atol=1e-8)
        np.testing.assert_allclose(state.tcp_pose.translation(), tcp_pos, atol=1e-10)
        np.testing.assert_allclose(state.tcp_pose.rotation_m(), tcp_rot, atol=1e-10)
        assert state.dt == 0.001


def test_gravity_follows_the_configured_direction():
    dynamics = common.PinocchioDynamics(fr3_xml(), False, FLANGE)
    q = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8])
    down = dynamics.compute(q, np.zeros(7)).gravity
    dynamics.set_gravity(np.array([0.0, 0.0, 9.81]))  # robot mounted upside down
    np.testing.assert_allclose(dynamics.compute(q, np.zeros(7)).gravity, -down, atol=1e-10)


def test_payload_can_be_replaced():
    dynamics = common.PinocchioDynamics(fr3_xml(), False, FLANGE)
    q, dq = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8]), np.zeros(7)
    without = dynamics.compute(q, dq).gravity
    dynamics.set_payload(PAYLOAD)
    assert not np.allclose(dynamics.compute(q, dq).gravity, without)
    dynamics.set_payload(common.Payload())
    np.testing.assert_allclose(dynamics.compute(q, dq).gravity, without, atol=1e-12)


def test_rejects_wrong_sizes_and_unknown_frame():
    with pytest.raises(RuntimeError):
        common.PinocchioDynamics(fr3_xml(), False, "no_such_frame")
    dynamics = common.PinocchioDynamics(fr3_xml(), False, FLANGE)
    assert dynamics.dof() == 7
    with pytest.raises(ValueError, match="q and dq"):
        dynamics.compute(np.zeros(6), np.zeros(6))
