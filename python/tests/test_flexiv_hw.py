"""The Flexiv extension's state and timing against a stand-in `flexivrdk`: the stamped fields on the robot and
gripper states, the optional 1 kHz poller with its history and drain, and impedance gains applied once per mode
switch instead of on every target. Skipped when the extension is not installed."""

import sys
import time
import types
from typing import Any

import numpy as np
import pytest

hw = pytest.importorskip("rcs_flexiv.hw")
from rcs._core.common import Pose

HOME = [0.0, -0.7, 0.0, 1.57, 0.0, 0.7, 0.0]


def stand_in():
    """A `flexivrdk` module that records every call; `states()` ticks the controller's clock on every read."""
    calls: list[tuple] = []
    modes = {
        n: types.SimpleNamespace(name=n)
        for n in ("IDLE", "NRT_JOINT_POSITION", "NRT_JOINT_IMPEDANCE", "NRT_CARTESIAN_MOTION_FORCE")
    }
    state = {"mode": modes["IDLE"], "enabled": False, "reads": 0}

    class Robot:
        def __init__(self, serial, *a):
            self.serial = serial

        def fault(self):
            return False

        def ClearFault(self):
            return True

        def estop_released(self):
            return True

        def Enable(self):
            calls.append(("Enable",))
            state["enabled"] = True

        def operational(self):
            return state["enabled"]

        def operational_status(self):
            return types.SimpleNamespace(name="READY")

        def info(self):
            return types.SimpleNamespace(
                DoF=7,
                serial_num=self.serial,
                model_name="Rizon4s",
                software_ver="v3.11.1",
                K_q_nom=[6000.0, 6000.0, 4200.0, 4200.0, 1500.0, 1500.0, 1500.0],
                K_x_nom=[6000.0, 6000.0, 6000.0, 300.0, 300.0, 300.0],
            )

        def mode(self):
            return state["mode"]

        def SwitchMode(self, mode):
            calls.append(("SwitchMode", mode.name))
            state["mode"] = mode

        def SetJointImpedance(self, K_q, Z_q=()):
            calls.append(("SetJointImpedance", list(K_q), list(Z_q)))

        def SetCartesianImpedance(self, K_x, Z_x=()):
            calls.append(("SetCartesianImpedance", list(K_x)))

        def SetMaxContactWrench(self, w):
            calls.append(("SetMaxContactWrench", list(w)))

        def SendJointPosition(self, q, dq, vmax, amax):
            calls.append(("SendJointPosition", list(q), list(vmax), list(amax)))

        def states(self):
            state["reads"] += 1
            n = state["reads"]
            return types.SimpleNamespace(
                timestamp=(1_700_000_000, n * 1_000_000),
                q=HOME,
                dq=[0.0] * 7,
                tau=[1.0] * 7,
                tau_ext=[0.0] * 7,
                tcp_pose=[0.5, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0],
                tcp_vel=[0.0] * 6,
                flange_pose=[0.5, 0.0, 0.45, 1.0, 0.0, 0.0, 0.0],
                ft_sensor_raw=[-3.7, 5.8, 3.7, 0.1, 0.1, -0.3],
                ext_wrench_in_tcp=[0.0] * 6,
                ext_wrench_in_world=[0.1] * 6,
                ext_wrench_in_tcp_raw=[0.0] * 6,
                ext_wrench_in_world_raw=[0.2] * 6,
            )

        def Stop(self):
            calls.append(("Stop",))

    class Tool:
        def __init__(self, robot):
            pass

        def name(self):
            return "grav"

        def params(self):
            return types.SimpleNamespace(mass=1.27, tcp_location=[0.0, 0.0, 0.15, 1.0, 0.0, 0.0, 0.0])

    class Gripper:
        def __init__(self, robot):
            pass

        def Enable(self, name):
            calls.append(("Gripper.Enable", name))

        def Init(self):
            calls.append(("Gripper.Init",))

        def Move(self, width, vel, force):
            calls.append(("Gripper.Move", width))

        def states(self):
            return types.SimpleNamespace(width=0.09, force=0.0, is_moving=False)

        def params(self):
            return types.SimpleNamespace(
                name="Grav", min_width=0.0, max_width=0.1, min_vel=0.01, max_vel=0.1, min_force=5.0, max_force=40.0
            )

    Mode = types.SimpleNamespace(**modes)
    return types.SimpleNamespace(Robot=Robot, Tool=Tool, Gripper=Gripper, Mode=Mode), calls, state


class _StubIk:
    """`Flexiv.__init__` compares the RDK's flange pose with the model's FK once; the stub answers the same pose."""

    def forward(self, q: Any, tcp_offset: Any) -> Pose:
        return Pose(translation=np.array([0.5, 0.0, 0.45]), quaternion=np.array([0.0, 0.0, 0.0, 1.0]))

    def inverse(self, pose: Any, q0: Any, tcp_offset: Any) -> Any:
        return np.asarray(HOME)


def config(**kw) -> "hw.FlexivConfig":
    return hw.FlexivConfig(
        robot_sn="Rizon4s-000000",
        control_mode=hw.FlexivControlMode.JOINT_IMPEDANCE,
        async_control=True,
        dof=7,
        joint_limits=np.array([[-3.0] * 7, [3.0] * 7]),
        q_home=np.asarray(HOME),
        tcp_offset=Pose(),
        attachment_site="flange",
        kinematic_model_path="rizon4s.urdf",
        startup_timeout=1.0,
        **kw,
    )


@pytest.fixture
def rdk(monkeypatch):
    mod, calls, state = stand_in()
    monkeypatch.setitem(sys.modules, "flexivrdk", mod)
    return mod, calls, state


def test_the_state_carries_the_controller_clock_the_read_time_and_the_raw_sensor(rdk):
    _, _, state = rdk
    robot = hw.Flexiv(config(), _StubIk())
    t0 = time.monotonic_ns()
    s = robot.get_state()
    t1 = time.monotonic_ns()
    assert t0 <= s.read_time_ns <= t1
    assert (
        s.timestamp == (1_700_000_000, state["reads"] * 1_000_000)
        and s.timestamp_ns == 1_700_000_000 * 10**9 + state["reads"] * 10**6
    )
    assert np.allclose(s.ft_sensor_raw, [-3.7, 5.8, 3.7, 0.1, 0.1, -0.3])
    assert np.allclose(s.tcp_wrench, 0.1) and np.allclose(s.tcp_wrench_raw, 0.2)
    assert np.allclose(s.tcp_pose, [0.5, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0]) and s.tcp_vel.shape == (6,)
    assert s.mode == "NRT_JOINT_IMPEDANCE" and s.operational and not s.fault
    g = hw.FlexivGripper(hw.FlexivGripperConfig(device_name="Flexiv-GN01"), robot).get_state()
    assert t1 <= g.read_time_ns <= time.monotonic_ns() and g.width == 0.09
    robot.close()


def test_the_gains_are_applied_on_connect_and_on_a_switch_not_on_every_target(rdk):
    mod, calls, state = rdk
    robot = hw.Flexiv(config(joint_stiffness=np.array([3000.0] * 7)), _StubIk())
    assert [c for c in calls if c[0] == "SetJointImpedance"] == [("SetJointImpedance", [3000.0] * 7, [])]
    n0 = len(calls)
    for _ in range(3):
        robot.set_joint_position(np.asarray(HOME))
    assert [c[0] for c in calls[n0:]] == ["SendJointPosition"] * 3  # no switch, no gains, no round trip per target
    robot.set_config(config(joint_stiffness=np.array([1500.0] * 7)))
    assert calls[-1] == ("SetJointImpedance", [1500.0] * 7, [])  # a new config re-applies them
    state["mode"] = mod.Mode.IDLE  # something else switched the mode: the next target switches back and re-applies
    robot.set_joint_position(np.asarray(HOME))
    assert [c[0] for c in calls[-3:]] == ["SwitchMode", "SetJointImpedance", "SendJointPosition"]
    robot.close()


def test_the_poller_keeps_a_history_and_drains_it_once(rdk):
    _, calls, _ = rdk
    robot = hw.Flexiv(config(state_poll_hz=1000, state_history_s=0.5), _StubIk())
    time.sleep(0.25)
    hist = robot.get_state_history()
    assert 100 <= len(hist) <= 500, len(hist)  # about 250 samples in 0.25 s; the buffer holds 500
    reads = [s.read_time_ns for s in hist]
    assert reads == sorted(reads) and len(set(reads)) == len(reads)
    assert (
        robot.get_state() is hist[-1] or robot.get_state().read_time_ns >= hist[-1].read_time_ns
    )  # served from the buffer
    first = robot.drain_state_history()
    assert len(first) >= len(hist) and first[-1].read_time_ns >= hist[-1].read_time_ns
    time.sleep(0.05)
    second = robot.drain_state_history()
    assert second and second[0].read_time_ns > first[-1].read_time_ns and 20 <= len(second) <= 120
    assert robot.get_state_history(3) == robot.get_state_history()[-3:]
    robot.close()
    assert robot._poller is None and calls[-1] == ("Stop",)


def test_without_the_poller_the_history_is_empty_and_each_getter_reads_the_rdk(rdk):
    _, _, state = rdk
    robot = hw.Flexiv(config(), _StubIk())
    reads = state["reads"]
    robot.get_state()
    robot.get_state()
    assert state["reads"] == reads + 2 and robot.get_state_history() == [] and robot.drain_state_history() == []
    robot.close()
