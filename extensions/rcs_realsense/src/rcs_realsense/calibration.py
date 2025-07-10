import logging
import typing
from queue import Queue
from time import sleep

import apriltag
import cv2
import numpy as np
from rcs.camera.hw import CalibrationStrategy
from rcs.camera.interface import Frame
from tqdm import tqdm

logger = logging.getLogger(__name__)


class FR3BaseArucoCalibration(CalibrationStrategy):
    """Calibration with a 3D printed aruco marker that fits around the vention's FR3 base mounting plate."""

    def __init__(self, camera_name: str):
        # base frame to camera, world to base frame
        self._extrinsics: np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]] | None = None
        self.camera_name = camera_name
        self.world_to_tag = np.array(
            [
                [0, -1, 0, 0.2],
                [-1, 0, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]
        )

    def calibrate(
        self,
        samples: Queue[Frame],
        intrinsics: np.ndarray[tuple[typing.Literal[3], typing.Literal[4]], np.dtype[np.float64]],
    ) -> bool:
        logger.info("Calibrating camera %s. Position it as you wish and press enter.", self.camera_name)
        input()
        tries = 3
        while samples.qsize() < samples.maxsize - 1 and tries > 0:
            logger.info("not enought frames in recorded, waiting 2 seconds...")
            tries = -1
            sleep(2)
        if tries == 0:
            logger.warning("Calibration failed, not enough frames arrived.")
            return False
        frames = []
        for _ in samples.qsize():
            frames.append(samples.get())

        _, cam_to_tag, _ = get_average_marker_pose(
            frames, intrinsics=intrinsics, calib_tag_id=9, show_live_window=False
        )

        world_to_cam = self.world_to_tag @ np.linalg.inv(cam_to_tag)
        self._extrinsics = world_to_cam
        return True

    def get_extrinsics(self) -> np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]] | None:
        return self._extrinsics


def get_average_marker_pose(
    samples,
    intrinsics,
    calib_tag_id,
    show_live_window,
):
    # create detector
    options = apriltag.DetectorOptions(families="tag25h9")
    detector = apriltag.Detector(options=options)

    # make while loop with tqdm
    poses = []

    for frame in tqdm(samples):

        # detect tags
        marker_det, pose = get_marker_pose(calib_tag_id, detector, intrinsics, frame)

        if marker_det is None:
            continue

        for corner in marker_det.corners:
            corner = corner.astype(int)
            cv2.circle(frame, tuple(corner), 5, (0, 0, 255), -1)

        poses.append(pose)

        last_frame = frame.copy()

        camera_matrix = intrinsics

        if show_live_window:
            cv2.drawFrameAxes(frame, camera_matrix, None, pose[:3, :3], pose[:3, 3], 0.1)
            # show frame
            cv2.imshow("frame", frame)

            # wait for key press
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    if show_live_window:
        cv2.destroyAllWindows()

    # calculate the average marker pose
    poses = np.array(poses)
    avg_pose = np.mean(poses, axis=0)
    logger.info(f"Average pose: {avg_pose}")

    # paint avg pose on last frame
    if show_live_window:
        cv2.drawFrameAxes(last_frame, camera_matrix, None, avg_pose[:3, :3], avg_pose[:3, 3], 0.1)  # type: ignore
    return last_frame, avg_pose, camera_matrix


def get_marker_pose(calib_tag_id, detector, intrinsics, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    # count detections
    n_det = 0
    marker_det = None
    for det in detections:
        if det.tag_id != calib_tag_id:
            continue
        n_det += 1
        marker_det = det

    if n_det > 1:
        raise ValueError("Expected 1 detection of tag id " f"{calib_tag_id}, got {n_det}.")

    if marker_det is None:
        return None, None

    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    cx = intrinsics[0, 2]
    cy = intrinsics[1, 2]

    pose, _, _ = detector.detection_pose(
        marker_det,
        camera_params=(
            fx,
            fy,
            cx,
            cy,
        ),
        tag_size=0.1,
    )

    return marker_det, pose
