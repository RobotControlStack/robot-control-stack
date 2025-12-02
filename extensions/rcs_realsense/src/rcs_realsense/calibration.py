import logging
import threading
import typing
from pathlib import Path
from time import sleep

import cv2
import diskcache as dc
import numpy as np
from pupil_apriltags import Detector
from rcs._core import common
from rcs.camera.hw import CalibrationStrategy
from rcs.camera.interface import Frame
from tqdm import tqdm

logger = logging.getLogger(__name__)


class FR3BaseArucoCalibration(CalibrationStrategy):
    """Calibration with a 3D printed aruco marker that fits around the vention's FR3 base mounting plate."""

    def __init__(self, camera_name: str):
        # base frame to camera, world to base frame
        self._cache = dc.Cache(Path.home() / ".cache" / "rcs")
        self._extrinsics: np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]] | None = (
            self._cache.get(f"{camera_name}_extrinsics")
        )  # None
        self.camera_name = camera_name
        self.tag_to_world = common.Pose(
            rpy_vector=np.array([np.pi, 0, -np.pi / 2]), translation=np.array([0.145, 0, 0])  # type: ignore
        ).pose_matrix()

    def calibrate(
        self,
        samples: list[Frame],
        intrinsics: np.ndarray[tuple[typing.Literal[3], typing.Literal[4]], np.dtype[np.float64]],
        lock: threading.Lock,
    ) -> bool:
        logger.info("Calibrating camera %s. Position it as you wish and press enter.", self.camera_name)
        input()
        tries = 3
        while len(samples) < 10 and tries > 0:
            logger.info("not enough frames in recorded, waiting 2 seconds...")
            tries = -1
            sleep(2)
        if tries == 0:
            logger.warning("Calibration failed, not enough frames arrived.")
            return False

        frames = []
        with lock:
            for sample in samples:
                frames.append(sample.camera.color.data.copy())
        # print(frames) # Removed print for cleaner logs, optional

        _, tag_to_cam = get_average_marker_pose(frames, intrinsics=intrinsics, calib_tag_id=9, show_live_window=False)

        cam_to_world = self.tag_to_world @ np.linalg.inv(tag_to_cam)
        world_to_cam = np.linalg.inv(cam_to_world)
        self._extrinsics = world_to_cam  # type: ignore
        self._cache.set(f"{self.camera_name}_extrinsics", world_to_cam, expire=3600)
        return True

    def get_extrinsics(self) -> np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]] | None:
        return self._extrinsics


def get_average_marker_pose(
    samples,
    intrinsics,
    calib_tag_id,
    show_live_window,
):
    # CHANGE 2: Simplified Initialization
    # No "DetectorOptions" object needed anymore.
    detector = Detector(families="tag25h9")

    # make while loop with tqdm
    poses = []

    last_frame = None
    for frame in tqdm(samples):

        # detect tags
        marker_det, pose = get_marker_pose(calib_tag_id, detector, intrinsics, frame)

        if marker_det is None:
            continue

        for corner in marker_det.corners:
            cv2.circle(frame, tuple(corner.astype(int)), 5, (0, 0, 255), -1)

        poses.append(pose)

        last_frame = frame.copy()

        camera_matrix = intrinsics[:3, :3]

        if show_live_window:
            # Note: pose[:3, :3] works because we construct the 4x4 matrix in get_marker_pose below
            cv2.drawFrameAxes(frame, camera_matrix, None, pose[:3, :3], pose[:3, 3], 0.1)  # type: ignore
            # show frame
            cv2.imshow("frame", frame)

            # wait for key press
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    if last_frame is None:
        msg = "No frames were processed, cannot calculate average pose. Check if the tag is visible."
        raise ValueError(msg)

    if show_live_window:
        cv2.destroyAllWindows()

    # calculate the average marker pose
    avg_pose = np.mean(poses, axis=0)
    logger.info(f"Average pose: {avg_pose}")

    return last_frame, avg_pose


def get_marker_pose(calib_tag_id, detector, intrinsics, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # CHANGE 3: Pose estimation happens INSIDE .detect()
    # We must extract camera params first to pass them here
    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    cx = intrinsics[0, 2]
    cy = intrinsics[1, 2]

    detections = detector.detect(gray, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=0.1)

    # count detections
    n_det = 0
    marker_det = None
    for det in detections:
        if det.tag_id != calib_tag_id:
            continue
        n_det += 1
        marker_det = det

    if n_det > 1:
        msg = f"Expected 1 detection of tag id {calib_tag_id}, got {n_det}."
        raise ValueError(msg)

    if marker_det is None:
        return None, None

    # CHANGE 4: Construct 4x4 Matrix manually
    # pupil-apriltags gives us R (3x3) and t (3x1). We must stack them.
    pose = np.eye(4)
    pose[:3, :3] = marker_det.pose_R
    pose[:3, 3] = marker_det.pose_t.ravel()  # ravel() flattens the (3,1) array to (3,)

    return marker_det, pose
