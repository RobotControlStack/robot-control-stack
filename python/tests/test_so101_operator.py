import time
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import numpy as np

operator_path = Path(__file__).resolve().parents[1] / "rcs" / "operator" / "so101.py"
spec = spec_from_file_location("rcs_local_operator_so101", operator_path)
assert spec is not None and spec.loader is not None
so101_module = module_from_spec(spec)
spec.loader.exec_module(so101_module)
SO101Operator = so101_module.SO101Operator
SO101OperatorConfig = so101_module.SO101OperatorConfig


class FakeLeader:
    def __init__(self, action):
        self._action = action
        self.disconnected = False

    def get_action(self):
        return self._action

    def disconnect(self):
        self.disconnected = True


def test_leader_action_to_target_converts_degrees_to_radians():
    joints, gripper = SO101Operator._leader_action_to_target(
        {
            "shoulder_pan.pos": 90.0,
            "shoulder_lift.pos": 0.0,
            "elbow_flex.pos": -45.0,
            "wrist_flex.pos": 30.0,
            "wrist_roll.pos": 180.0,
            "gripper.pos": 25.0,
        },
        use_degrees=True,
    )

    np.testing.assert_allclose(joints, np.deg2rad([90.0, 0.0, -45.0, 30.0, 180.0]))
    assert gripper == 0.25


def test_leader_action_to_target_keeps_non_degree_joint_values():
    joints, gripper = SO101Operator._leader_action_to_target(
        {
            "shoulder_pan.pos": -100.0,
            "shoulder_lift.pos": -50.0,
            "elbow_flex.pos": 0.0,
            "wrist_flex.pos": 50.0,
            "wrist_roll.pos": 100.0,
            "gripper.pos": 100.0,
        },
        use_degrees=False,
    )

    np.testing.assert_allclose(joints, np.array([-100.0, -50.0, 0.0, 50.0, 100.0]))
    assert gripper == 1.0


def test_operator_run_updates_latest_action_and_closes_cleanly():
    fake_leader = FakeLeader(
        {
            "shoulder_pan.pos": 10.0,
            "shoulder_lift.pos": 20.0,
            "elbow_flex.pos": 30.0,
            "wrist_flex.pos": 40.0,
            "wrist_roll.pos": 50.0,
            "gripper.pos": 60.0,
        }
    )
    cfg = SO101OperatorConfig(
        read_frequency=200,
        leader_factory=lambda _: fake_leader,
    )
    operator = SO101Operator(cfg)

    operator.start()
    time.sleep(0.05)
    action = operator.consume_action()
    operator.close()

    assert cfg.controller_name in action
    np.testing.assert_allclose(
        action[cfg.controller_name]["joints"],
        np.deg2rad([10.0, 20.0, 30.0, 40.0, 50.0]),
    )
    np.testing.assert_allclose(action[cfg.controller_name]["gripper"], np.array([0.6], dtype=np.float32))
    assert fake_leader.disconnected
