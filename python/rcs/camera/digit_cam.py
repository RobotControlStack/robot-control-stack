import threading
import time
from typing import Optional

from digit_interface.digit import Digit
from rcs._core.common import BaseCameraConfig
from rcs.camera.hw import HardwareCamera
from rcs.camera.interface import CameraFrame, DataFrame, Frame


class DigitCam(HardwareCamera):
    """
    Interface for DIGIT cameras.

    Each camera has a dedicated background thread that continuously pulls frames
    using digit.get_frame() and writes the latest complete image into a
    lock-protected buffer.

    poll_frame() returns the most recent buffered frame instead of calling
    digit.get_frame() directly.
    """

    def __init__(self, cameras: dict[str, BaseCameraConfig]):
        self.cameras = cameras
        self._camera_names = list(self.cameras.keys())

        self._cameras: dict[str, Digit] = {}

        # One latest-frame buffer per camera.
        self._frame_buffers: dict[str, Optional[object]] = {
            name: None for name in self._camera_names
        }

        # One mutex per camera to protect its buffer.
        self._frame_locks: dict[str, threading.Lock] = {
            name: threading.Lock() for name in self._camera_names
        }

        # One condition per camera so poll_frame can wait for first frame.
        self._frame_ready: dict[str, threading.Condition] = {
            name: threading.Condition(self._frame_locks[name])
            for name in self._camera_names
        }

        # One worker thread per camera.
        self._camera_getframe_threads: dict[str, threading.Thread] = {}

        # Shared shutdown signal.
        self._stop_event = threading.Event()

        # Optional: store worker exceptions so poll_frame can report them.
        self._thread_errors: dict[str, BaseException] = {}

    def open(self):
        """
        Initialize the DIGIT cameras and start one get_frame thread per camera.
        """
        self._stop_event.clear()
        self._thread_errors.clear()

        for name, camera in self.cameras.items():
            digit = Digit(camera.identifier, name)
            digit.connect()

            qvga_res = Digit.STREAMS["QVGA"]
            digit.set_resolution(qvga_res)

            fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
            digit.set_fps(fps_30)

            self._cameras[name] = digit

        # Start polling threads only after all cameras are configured.
        for name, digit in self._cameras.items():
            thread = threading.Thread(
                target=self._get_frame_loop,
                args=(name, digit),
                name=f"DigitCamGetFrame-{name}",
                daemon=True,
            )
            self._camera_getframe_threads[name] = thread
            thread.start()

    def _get_frame_loop(self, camera_name: str, digit: Digit):
        """
        Background worker for one camera.

        Continuously reads frames from the DIGIT device and updates the latest
        buffered frame under a mutex.
        """
        try:
            while not self._stop_event.is_set():
                frame = digit.get_frame()

                # Copy before storing so the buffer owns stable frame memory.
                # This matters if the DIGIT SDK reuses internal buffers.
                frame = frame.copy()

                condition = self._frame_ready[camera_name]

                with condition:
                    self._frame_buffers[camera_name] = frame
                    condition.notify_all()

        except BaseException as exc:
            self._thread_errors[camera_name] = exc

            condition = self._frame_ready[camera_name]
            with condition:
                condition.notify_all()

    @property
    def camera_names(self) -> list[str]:
        """Returns the names of the cameras in this set."""
        return self._camera_names

    def poll_frame(self, camera_name: str) -> Frame:
        """
        Returns the latest buffered frame from the camera with the given name.

        This does not call digit.get_frame() directly.
        """
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

            # Copy on read so the caller cannot observe the writer mutating
            # the same ndarray object later.
            frame = self._frame_buffers[camera_name].copy()

        # DIGIT frame appears to be RGB.
        # Convert RGB -> BGR as expected by OpenCV.
        #
        # The final .copy() is important because frame[:, :, ::-1] creates
        # a negative-stride view, which can cause issues with some consumers.
        bgr_frame = frame[:, :, ::-1].copy()

        color = DataFrame(data=bgr_frame)
        cf = CameraFrame(color=color)

        return Frame(camera=cf)

    def close(self):
        """
        Stop get_frame threads and close DIGIT devices.
        """
        self._stop_event.set()

        # Wake any poll_frame calls waiting for first frame.
        for condition in self._frame_ready.values():
            with condition:
                condition.notify_all()

        # Join threads before disconnecting devices.
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
        """No calibration needed for DIGIT cameras."""
        return True