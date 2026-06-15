import threading
import time
from typing import Optional

import numpy as np

from digit_interface.digit import Digit
from rcs._core.common import BaseCameraConfig
from rcs.camera.hw import HardwareCamera
from rcs.camera.interface import CameraFrame, DataFrame, Frame


class DigitCam(HardwareCamera):
    """
    DIGIT camera interface with one acquisition thread per camera.

    Each camera thread continuously calls digit.get_frame(), validates the image,
    and only publishes frames that pass the sanity check.
    """

    # Optional global lock.
    #
    # Enable this if the DIGIT SDK appears to use shared internal buffers or is
    # not thread-safe across multiple Digit objects.
    _global_digit_get_frame_lock = threading.Lock()

    def __init__(
        self,
        cameras: dict[str, BaseCameraConfig],
        flush_frames_on_open: int = 10,
        frame_timeout_s: float = 2.0,
        max_bad_frames_before_warning: int = 30,
        serialize_get_frame: bool = True,
    ):
        self.cameras = cameras
        self._camera_names = list(self.cameras.keys())

        self.flush_frames_on_open = flush_frames_on_open
        self.frame_timeout_s = frame_timeout_s
        self.max_bad_frames_before_warning = max_bad_frames_before_warning
        self.serialize_get_frame = serialize_get_frame

        self._cameras: dict[str, Digit] = {}

        self._frame_buffers: dict[str, Optional[np.ndarray]] = {
            name: None for name in self._camera_names
        }

        self._frame_locks: dict[str, threading.Lock] = {
            name: threading.Lock() for name in self._camera_names
        }

        self._frame_ready: dict[str, threading.Condition] = {
            name: threading.Condition(self._frame_locks[name])
            for name in self._camera_names
        }

        self._camera_getframe_threads: dict[str, threading.Thread] = {}

        self._stop_event = threading.Event()
        self._thread_errors: dict[str, BaseException] = {}

        self._bad_frame_counts: dict[str, int] = {
            name: 0 for name in self._camera_names
        }

        self._published_frame_counts: dict[str, int] = {
            name: 0 for name in self._camera_names
        }

    def open(self):
        self._stop_event.clear()
        self._thread_errors.clear()

        for name in self._camera_names:
            self._frame_buffers[name] = None
            self._bad_frame_counts[name] = 0
            self._published_frame_counts[name] = 0

        for name, camera in self.cameras.items():
            digit = Digit(camera.identifier, name)
            digit.connect()

            qvga_res = Digit.STREAMS["QVGA"]
            digit.set_resolution(qvga_res)

            fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
            digit.set_fps(fps_30)

            self._cameras[name] = digit

        for name, digit in self._cameras.items():
            thread = threading.Thread(
                target=self._get_frame_loop,
                args=(name, digit),
                name=f"DigitCamGetFrame-{name}",
                daemon=True,
            )
            self._camera_getframe_threads[name] = thread
            thread.start()

        for name in self._camera_names:
            self._wait_until_camera_ready(name)

    def _safe_get_frame(self, digit: Digit):
        """
        Calls digit.get_frame().

        If serialize_get_frame=True, all DIGIT get_frame calls are serialized
        through one global mutex. This can help if the DIGIT SDK has shared
        internal state or shared frame buffers.
        """
        if self.serialize_get_frame:
            with DigitCam._global_digit_get_frame_lock:
                return digit.get_frame()

        return digit.get_frame()

    def _get_frame_loop(self, camera_name: str, digit: Digit):
        try:
            discarded = 0
            expected_shape = None

            while not self._stop_event.is_set():
                frame = self._safe_get_frame(digit)

                if frame is None:
                    self._bad_frame_counts[camera_name] += 1
                    continue

                if not isinstance(frame, np.ndarray):
                    frame = np.asarray(frame)

                if expected_shape is None and self._basic_frame_check(frame):
                    expected_shape = frame.shape

                if expected_shape is not None and frame.shape != expected_shape:
                    self._bad_frame_counts[camera_name] += 1
                    continue

                if not self._is_good_frame(frame):
                    self._bad_frame_counts[camera_name] += 1

                    if (
                        self._bad_frame_counts[camera_name]
                        == self.max_bad_frames_before_warning
                    ):
                        print(
                            f"[DigitCam] Warning: camera '{camera_name}' has produced "
                            f"{self._bad_frame_counts[camera_name]} bad frames."
                        )

                    continue

                if discarded < self.flush_frames_on_open:
                    discarded += 1
                    continue

                frame = frame.copy()

                condition = self._frame_ready[camera_name]
                with condition:
                    self._frame_buffers[camera_name] = frame
                    self._published_frame_counts[camera_name] += 1
                    condition.notify_all()

        except BaseException as exc:
            self._thread_errors[camera_name] = exc

            condition = self._frame_ready[camera_name]
            with condition:
                condition.notify_all()

    def _basic_frame_check(self, frame: np.ndarray) -> bool:
        """
        Basic structural checks.
        """
        if frame is None:
            return False

        if not isinstance(frame, np.ndarray):
            return False

        if frame.ndim != 3:
            return False

        if frame.shape[2] != 3:
            return False

        if frame.size == 0:
            return False

        return True

    def _is_good_frame(self, frame: np.ndarray) -> bool:
        """
        Returns False for obviously broken/torn frames.

        This catches cases where the image has a strong vertical discontinuity,
        like one half of the image coming from a different buffer/exposure.

        This is intentionally conservative: it should reject frames with a huge
        seam while accepting normal DIGIT images with smooth lighting gradients.
        """
        if not self._basic_frame_check(frame):
            return False

        if not np.isfinite(frame).all():
            return False

        h, w, c = frame.shape

        if h < 8 or w < 8:
            return False

        img = frame.astype(np.float32)

        # Check for vertical tearing.
        #
        # Compute per-column discontinuity:
        # difference between column x and column x - 1, averaged over rows/channels.
        vertical_edges = np.mean(
            np.abs(img[:, 1:, :] - img[:, :-1, :]),
            axis=(0, 2),
        )

        median_edge = float(np.median(vertical_edges))
        max_edge = float(np.max(vertical_edges))
        max_edge_x = int(np.argmax(vertical_edges)) + 1

        # Avoid divide-by-zero on very flat frames.
        edge_ratio = max_edge / max(median_edge, 1.0)

        # Tearing usually creates one extremely dominant vertical seam.
        #
        # In your example, this seam is around the center of the image.
        # We do not require it to be exactly centered, but we reject only if the
        # dominant seam is very strong.
        has_dominant_vertical_seam = edge_ratio > 8.0 and max_edge > 25.0

        if has_dominant_vertical_seam:
            return False

        # Optional horizontal tearing check too.
        horizontal_edges = np.mean(
            np.abs(img[1:, :, :] - img[:-1, :, :]),
            axis=(1, 2),
        )

        median_h_edge = float(np.median(horizontal_edges))
        max_h_edge = float(np.max(horizontal_edges))
        h_edge_ratio = max_h_edge / max(median_h_edge, 1.0)

        has_dominant_horizontal_seam = h_edge_ratio > 8.0 and max_h_edge > 25.0

        if has_dominant_horizontal_seam:
            return False

        return True

    def _wait_until_camera_ready(self, camera_name: str):
        condition = self._frame_ready[camera_name]
        deadline = time.monotonic() + self.frame_timeout_s

        with condition:
            while self._frame_buffers[camera_name] is None:
                if camera_name in self._thread_errors:
                    raise RuntimeError(
                        f"Frame thread for camera '{camera_name}' failed"
                    ) from self._thread_errors[camera_name]

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Timed out waiting for a valid frame from camera "
                        f"'{camera_name}'. Bad frames seen: "
                        f"{self._bad_frame_counts[camera_name]}"
                    )

                condition.wait(timeout=min(0.1, remaining))

    @property
    def camera_names(self) -> list[str]:
        return self._camera_names

    def poll_frame(self, camera_name: str) -> Frame:
        if camera_name not in self._cameras:
            raise KeyError(f"Unknown or unopened camera: {camera_name}")

        condition = self._frame_ready[camera_name]

        with condition:
            while self._frame_buffers[camera_name] is None:
                if camera_name in self._thread_errors:
                    raise RuntimeError(
                        f"Frame thread for camera '{camera_name}' failed"
                    ) from self._thread_errors[camera_name]

                if self._stop_event.is_set():
                    raise RuntimeError(
                        f"Camera '{camera_name}' was closed before a frame was available"
                    )

                condition.wait(timeout=0.1)

            frame = self._frame_buffers[camera_name].copy()

        # RGB -> BGR for OpenCV consumers.
        bgr_frame = frame[:, :, ::-1].copy()

        color = DataFrame(data=bgr_frame)
        cf = CameraFrame(color=color)

        return Frame(camera=cf)

    def close(self):
        self._stop_event.set()

        for condition in self._frame_ready.values():
            with condition:
                condition.notify_all()

        for thread in self._camera_getframe_threads.values():
            thread.join(timeout=2.0)

        self._camera_getframe_threads.clear()

        for digit in self._cameras.values():
            digit.disconnect()

        self._cameras = {}

        for name in self._frame_buffers:
            self._frame_buffers[name] = None

    def config(self, camera_name) -> BaseCameraConfig:
        return self.cameras[camera_name]

    def calibrate(self) -> bool:
        return True