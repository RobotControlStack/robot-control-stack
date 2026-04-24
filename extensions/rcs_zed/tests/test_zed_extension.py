import sys
from pathlib import Path
from typing import TypedDict

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "python"))
sys.path.insert(0, str(REPO_ROOT / "extensions/rcs_zed/src"))

from rcs_zed.camera import ZEDCameraSet, ZEDDeviceInfo, ZEDFrameBundle  # noqa: E402

from rcs import common  # noqa: E402


class FakeOpenedZEDCamera:
    def __init__(self, device_info: ZEDDeviceInfo, color_intrinsics: np.ndarray, frame_bundle: ZEDFrameBundle):
        self.device_info = device_info
        self.color_intrinsics = color_intrinsics
        self._frame_bundle = frame_bundle
        self.closed = False

    def grab_frame(self) -> ZEDFrameBundle:
        return self._frame_bundle

    def close(self):
        self.closed = True


class PatchZedState(TypedDict):
    devices: dict[str, ZEDDeviceInfo]
    opened: dict[str, FakeOpenedZEDCamera]
    open_calls: list[tuple[str, bool, bool]]


@pytest.fixture()
def patch_zed(monkeypatch) -> PatchZedState:
    state: PatchZedState = {"devices": {}, "opened": {}, "open_calls": []}

    def fake_enumerate(_cls):
        return state["devices"]

    def fake_open(_cls, config: common.BaseCameraConfig, *, enable_depth: bool, enable_imu: bool):
        state["open_calls"].append((config.identifier, enable_depth, enable_imu))
        return state["opened"][config.identifier]

    monkeypatch.setattr(ZEDCameraSet, "enumerate_connected_devices", classmethod(fake_enumerate))
    monkeypatch.setattr(ZEDCameraSet, "open_camera", classmethod(fake_open))
    return state


def test_zed_frame_mapping_depth_scaling_and_imu_downgrade(patch_zed):
    intrinsics = np.array([[100.0, 0.0, 10.0, 0.0], [0.0, 110.0, 20.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
    color = np.arange(27, dtype=np.uint8).reshape(3, 3, 3)
    depth = np.full((3, 3), 1234, dtype=np.uint16)
    frame_bundle = ZEDFrameBundle(
        color=color,
        depth=depth,
        accel=None,
        gyro=None,
        timestamp=12.5,
        color_intrinsics=intrinsics,
    )
    device_info = ZEDDeviceInfo(serial="123", model="ZED Mini", has_depth=True, has_imu=False)
    opened = FakeOpenedZEDCamera(device_info=device_info, color_intrinsics=intrinsics, frame_bundle=frame_bundle)
    patch_zed["devices"] = {"123": device_info}
    patch_zed["opened"] = {"123": opened}

    camera_set = ZEDCameraSet(
        cameras={
            "wrist": common.BaseCameraConfig(
                identifier="123", resolution_width=1280, resolution_height=720, frame_rate=30
            )
        },
    )
    camera_set.open()
    frame = camera_set.poll_frame("wrist")
    assert frame.camera.color.intrinsics is not None

    assert patch_zed["open_calls"] == [("123", True, False)]
    assert np.array_equal(frame.camera.color.data, color)
    assert np.array_equal(frame.camera.depth.data, depth)  # type: ignore[union-attr]
    assert np.array_equal(frame.camera.color.intrinsics, intrinsics)
    assert frame.imu is None
    assert frame.avg_timestamp == 12.5


def test_zed_enumeration_and_multi_camera_open(patch_zed):
    intrinsics = np.eye(3, 4)
    bundle = ZEDFrameBundle(
        color=np.zeros((2, 2, 3), dtype=np.uint8), depth=None, timestamp=1.0, color_intrinsics=intrinsics
    )
    devices = {
        "111": ZEDDeviceInfo(serial="111", model="ZED Mini", has_depth=True, has_imu=False),
        "222": ZEDDeviceInfo(serial="222", model="ZED 2", has_depth=True, has_imu=True),
    }
    opened = {serial: FakeOpenedZEDCamera(info, intrinsics, bundle) for serial, info in devices.items()}
    patch_zed["devices"] = devices
    patch_zed["opened"] = opened

    enumerated = ZEDCameraSet.enumerate_connected_devices()
    assert enumerated == devices

    camera_set = ZEDCameraSet(
        cameras={
            "left": common.BaseCameraConfig(
                identifier="111", resolution_width=1280, resolution_height=720, frame_rate=30
            ),
            "right": common.BaseCameraConfig(
                identifier="222", resolution_width=1280, resolution_height=720, frame_rate=30
            ),
        },
    )
    camera_set.open()
    assert patch_zed["open_calls"] == [("111", True, False), ("222", True, True)]
    camera_set.close()
    assert opened["111"].closed
    assert opened["222"].closed
