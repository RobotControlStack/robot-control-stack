from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[3]
for candidate in (REPO_ROOT / "python", REPO_ROOT / "extensions" / "rcs_sb3", REPO_ROOT / "extensions" / "rcs_sb3" / "src"):
    sys.path.insert(0, str(candidate))

import cv2
import numpy as np

from run_test import _EpisodeVideoWriter, _compose_video_frame


def test_compose_video_frame_and_write_mp4(tmp_path: Path):
    obs = {
        "frames": {
            "head": {"rgb": {"data": np.full((240, 320, 3), (0, 0, 255), dtype=np.uint8)}},
            "tactile_gripperleft_digit_pad": {
                "rgb": {"data": np.full((240, 320, 3), (255, 0, 0), dtype=np.uint8)}
            },
            "tactile_gripperright_digit_pad": {
                "rgb": {"data": np.full((240, 320, 3), (0, 255, 0), dtype=np.uint8)}
            },
        }
    }
    composed = _compose_video_frame(obs)

    assert composed.shape == (240, 960, 3)
    assert composed.dtype == np.uint8

    video_path = tmp_path / "taxim_inference.mp4"
    writer = _EpisodeVideoWriter(video_path, fps=30.0)
    writer.write(composed)
    writer.write(composed)
    writer.close()

    assert video_path.exists()
    assert video_path.stat().st_size > 0

    cap = cv2.VideoCapture(str(video_path))
    try:
        ok, frame = cap.read()
        assert ok is True
        assert frame is not None
        assert frame.shape[0] == 240
        assert frame.shape[1] == 960
    finally:
        cap.release()
