from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

import rcs
from rcs import common
from rcs_robotiq2f85 import kinematics
from rcs_robotiq2f85.kinematics import robotiq_2f85_finger_pose_offsets_from_sites, robotiq_2f85_finger_poses

ROBOTIQ_2F85_MODEL_PATH = (
    Path(__file__).resolve().parents[3]
    / "assets"
    / "grippers"
    / "robotiq_2f85"
    / "robotiq_2f85.xml"
)


def test_robotiq_2f85_pose_cache_builds_from_open_to_closed(monkeypatch, caplog):
    commands: list[float] = []

    class FakeModel:
        actuator_ctrlrange = np.array([[0.0, 255.0]])

        @staticmethod
        def actuator(name: str) -> SimpleNamespace:
            assert name == "fingers_actuator"
            return SimpleNamespace(id=0)

    class FakeData:
        def __init__(self, model: FakeModel):
            del model
            self.ctrl = np.zeros(1)
            self.qvel = np.zeros(1)

    def from_xml_path(path: str) -> FakeModel:
        del path
        return FakeModel()

    def step(model: FakeModel, data: FakeData) -> None:
        del model
        commands.append(float(data.ctrl[0]))
        data.qvel[:] = 1.0 if np.isclose(data.ctrl[0], 127.5) else 0.0

    fake_mujoco = SimpleNamespace(
        MjModel=SimpleNamespace(from_xml_path=from_xml_path),
        MjData=FakeData,
        mj_step=step,
    )

    def pose_from_control(data: FakeData, name: str) -> common.Pose:
        del name
        return common.Pose(translation=np.array([data.ctrl[0], 0.0, 0.0]))

    monkeypatch.setattr(kinematics, "mujoco", fake_mujoco)
    monkeypatch.setattr(kinematics, "_pose_from_body", pose_from_control)
    poses = kinematics._build_robotiq_2f85_pose_cache("unused.xml")

    assert commands[0] == 0.0
    assert commands[-1] == 255.0
    assert poses[-1, 0, 0, 3] == 0.0
    assert poses[0, 0, 0, 3] == 255.0
    assert np.array_equal(poses[500], poses[501])
    assert "failed to converge at state 0.500" in caplog.text
    assert "reusing pose from state 0.501" in caplog.text


def test_robotiq_2f85_pose_cache_persists_across_memory_caches(tmp_path, monkeypatch):
    model_path = tmp_path / "robotiq.xml"
    model_path.write_text("<mujoco/>")
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    expected = np.zeros(kinematics._ROBOTIQ_2F85_CACHE_SHAPE, dtype=np.float64)
    expected[:, :, 3, 3] = 1.0
    build_calls = 0

    def build_cache(path: str) -> np.ndarray:
        nonlocal build_calls
        assert path == str(model_path.resolve())
        build_calls += 1
        return expected

    monkeypatch.setattr(kinematics, "_build_robotiq_2f85_pose_cache", build_cache)
    kinematics._robotiq_2f85_pose_cache.cache_clear()
    first = kinematics._robotiq_2f85_pose_cache(str(model_path))

    kinematics._robotiq_2f85_pose_cache.cache_clear()
    second = kinematics._robotiq_2f85_pose_cache(str(model_path))

    assert build_calls == 1
    assert np.array_equal(first, expected)
    assert np.array_equal(second, expected)
    assert not second.flags.writeable
    assert len(list((tmp_path / "cache" / "rcs" / "robotiq2f85").glob("*.npy"))) == 1


def test_robotiq_2f85_pose_cache_rebuilds_invalid_disk_cache(tmp_path, monkeypatch):
    model_path = tmp_path / "robotiq.xml"
    model_path.write_text("<mujoco/>")
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    cache_path = kinematics._robotiq_2f85_cache_path(model_path.resolve())
    cache_path.parent.mkdir(parents=True)
    np.save(cache_path, np.zeros((1,), dtype=np.float64))
    expected = np.ones(kinematics._ROBOTIQ_2F85_CACHE_SHAPE, dtype=np.float64)

    def rebuild_cache(path: str) -> np.ndarray:
        del path
        return expected

    monkeypatch.setattr(kinematics, "_build_robotiq_2f85_pose_cache", rebuild_cache)
    kinematics._robotiq_2f85_pose_cache.cache_clear()
    poses = kinematics._robotiq_2f85_pose_cache(str(model_path))

    assert np.array_equal(poses, expected)
    assert np.load(cache_path, allow_pickle=False).shape == kinematics._ROBOTIQ_2F85_CACHE_SHAPE


def test_robotiq_2f85_pose_cache_is_invalidated_when_model_changes(tmp_path, monkeypatch):
    model_path = tmp_path / "robotiq.xml"
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    model_path.write_text("<mujoco model='first'/>")
    first_cache_path = kinematics._robotiq_2f85_cache_path(model_path.resolve())

    model_path.write_text("<mujoco model='second'/>")
    second_cache_path = kinematics._robotiq_2f85_cache_path(model_path.resolve())

    assert first_cache_path != second_cache_path


@pytest.mark.parametrize(
    ("normalized_command", "left_position", "right_position"),
    [
        (0.0, [-0.007465, -0.00905, 0.129130], [0.007466, 0.00905, 0.129130]),
        (0.5, [-0.030551, -0.00905, 0.126510], [0.030551, 0.00905, 0.126510]),
        (1.0, [-0.050800, -0.00905, 0.114817], [0.050800, 0.00905, 0.114817]),
    ],
)
def test_robotiq_2f85_fingertip_fk(normalized_command, left_position, right_position):
    """Look up follower FK from a normalized command: 0 closed, 1 open."""
    poses = robotiq_2f85_finger_poses(normalized_command, model_path=ROBOTIQ_2F85_MODEL_PATH)

    expected_left_pose = common.Pose(
        rotation=np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]),
        translation=np.array(left_position),
    )
    expected_right_pose = common.Pose(
        rotation=np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]),
        translation=np.array(right_position),
    )

    assert poses.left.is_close(expected_left_pose, eps_r=1e-3, eps_t=1e-4)
    assert poses.right.is_close(expected_right_pose, eps_r=1e-3, eps_t=1e-4)


def test_robotiq_2f85_fingertip_fk_with_custom_mount_offsets():
    # The fixed body/site transforms match robotiq_2f85_digit_new.xml.
    custom_mount_model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="left_follower">
              <body name="left_digit" euler="-1.571 0 0" pos="-0.0112 0.032 -0.00905">
                <site name="left_digit_pad" pos="0.02 0 0.016" euler="3.142 1.571 0"/>
              </body>
            </body>
            <body name="right_follower">
              <body name="right_digit" euler="-1.571 0 0" pos="-0.0112 0.032 -0.00905">
                <site name="right_digit_pad" pos="0.02 0 0.016" euler="3.142 1.571 0"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    custom_mount_data = mujoco.MjData(custom_mount_model)
    offsets = robotiq_2f85_finger_pose_offsets_from_sites(
        custom_mount_model,
        custom_mount_data,
        left_site="left_digit_pad",
        right_site="right_digit_pad",
    )

    poses = robotiq_2f85_finger_poses(0.5, offsets=offsets, model_path=ROBOTIQ_2F85_MODEL_PATH)

    expected_left_pose = common.Pose(
        rotation=np.array(
            [
                [-0.0001713, 0.0, 1.0],
                [-0.0002037, -1.0, 0.0],
                [1.0, -0.0002037, 0.0001713],
            ]
        ),
        translation=np.array([-0.021750, 0.0000033, 0.174510]),
    )
    expected_right_pose = common.Pose(
        rotation=np.array(
            [
                [0.0002066, 0.0, -1.0],
                [0.0002037, 1.0, 0.0],
                [1.0, -0.0002037, 0.0002066],
            ]
        ),
        translation=np.array([0.021752, -0.0000033, 0.174510]),
    )

    assert poses.left.is_close(expected_left_pose, eps_r=1e-3, eps_t=1e-4)
    assert poses.right.is_close(expected_right_pose, eps_r=1e-3, eps_t=1e-4)
