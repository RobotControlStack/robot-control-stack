import datetime
import logging
import math
import subprocess
from pathlib import Path
from time import perf_counter, sleep

import duckdb
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class SimpleFrameRate:
    def __init__(self, frame_rate: float | None, loop_name: str = "SimpleFrameRate"):
        """SimpleFrameRate is a utility class to manage frame rates in a simple way.
        It allows you to call it in a loop, and it will sleep the necessary time to maintain the desired frame rate.

        Args:
            frame_rate (float): The desired frame rate in frames per second.
        """
        self.t: float | None = None
        self._last_print: float | None = None
        self.frame_rate = frame_rate
        self.loop_name = loop_name

    def reset(self):
        self.t = None

    def __call__(self):
        if self.frame_rate is None:
            return
        if self.t is None:
            self.t = perf_counter()
            self._last_print = self.t
            return
        sleep_time = 1 / self.frame_rate - (perf_counter() - self.t)
        if sleep_time > 0:
            sleep(sleep_time)
        if self._last_print is None or perf_counter() - self._last_print > 10:
            self._last_print = perf_counter()
            logger.debug(f"FPS {self.loop_name}: {1 / (perf_counter() - self.t)}")

        self.t = perf_counter()


class ContextManager:
    def __enter__(self):
        pass

    def __exit__(self, *args):
        pass


def _write_mp4(frames: list[np.ndarray], output_path: Path, fps: int) -> None:
    if not frames:
        return

    height, width = frames[0].shape[:2]
    process = subprocess.Popen(
        [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(fps),
            "-i",
            "-",
            "-an",
            "-vf",
            "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output_path),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    assert process.stdin is not None
    for frame in frames:
        process.stdin.write(np.ascontiguousarray(frame).astype(np.uint8).tobytes())
    process.stdin.close()
    process.wait()


def _render_action_panel(
    joint_history: dict[str, np.ndarray],
    gripper_history: dict[str, np.ndarray],
    step: int,
    height: int,
    width: int,
) -> np.ndarray:
    from matplotlib import pyplot as plt

    fig, axes = plt.subplots(
        len(joint_history),
        1,
        figsize=(width / 100, height / 100),
        dpi=100,
        squeeze=False,
    )
    x = np.arange(len(next(iter(joint_history.values()))))
    for axis, robot_key in zip(axes[:, 0], joint_history, strict=True):
        joints = joint_history[robot_key]
        for joint_idx in range(joints.shape[1]):
            axis.plot(x, joints[:, joint_idx], linewidth=1)
        axis.axvline(step, color="tab:red", linewidth=2)
        axis.set_xlim(0, max(len(x) - 1, 1))
        axis.set_title(robot_key)
        axis.set_xlabel("step")
        axis.set_ylabel("joint")

        gripper_axis = axis.twinx()
        gripper_axis.plot(x, gripper_history[robot_key], color="black", linestyle="--", linewidth=2)
        gripper_axis.set_ylabel("gripper")
        gripper_axis.set_ylim(-0.1, 1.1)

    fig.tight_layout()
    fig.canvas.draw()
    image = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
    plt.close(fig)
    return image


def export_episode_videos(
    dataset: str | Path,
    output: str | Path,
    fps: int = 30,
    n: int = -1,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import torch
    from torchvision.io import decode_jpeg

    dataset = Path(dataset)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)

    source = str(dataset / "*.parquet") if dataset.is_dir() else str(dataset)
    source_escaped = source.replace("'", "''")
    conn = duckdb.connect()
    relation = conn.sql(f"SELECT * FROM read_parquet('{source_escaped}')")
    frame_struct = relation.select("obs.frames").types[0]
    action_struct = relation.select("action").types[0]
    camera_names = [name for name, _ in frame_struct.children]
    robot_names = [name for name, _ in relation.select("obs").types[0].children if name != "frames"]
    action_fields_by_robot = {
        robot: {field_name for field_name, _ in robot_struct.children} for robot, robot_struct in action_struct.children
    }

    uuids = conn.execute(f"SELECT DISTINCT uuid FROM read_parquet('{source_escaped}') ORDER BY uuid").fetchall()
    for index, (episode_id,) in enumerate(uuids):
        if n != -1 and index >= n:
            break

        image_selects = ", ".join(f"obs.frames.{camera}.rgb.data AS {camera}" for camera in camera_names)
        joint_selects = ", ".join(
            (
                f"COALESCE(action.{robot}.joints, obs.{robot}.joints) AS joints_{robot}"
                if "joints" in action_fields_by_robot.get(robot, set())
                else f"obs.{robot}.joints AS joints_{robot}"
            )
            for robot in robot_names
        )
        gripper_selects = ", ".join(
            (
                f"COALESCE(CAST(action.{robot}.gripper[1] AS DOUBLE), obs.{robot}.gripper[1]) AS gripper_{robot}"
                if "gripper" in action_fields_by_robot.get(robot, set())
                else f"obs.{robot}.gripper[1] AS gripper_{robot}"
            )
            for robot in robot_names
        )
        state_selects = ", ".join(
            [
                joint_selects,
                gripper_selects,
            ]
        )
        not_null_checks = " ".join(f"AND obs.frames.{camera}.rgb.data IS NOT NULL" for camera in camera_names)
        rows = conn.execute(
            f"""
            SELECT timestamp, {image_selects}, {state_selects}
            FROM read_parquet('{source_escaped}')
            WHERE uuid = ?
              {not_null_checks}
            ORDER BY step
            """,
            [episode_id],
        ).fetchall()
        if not rows:
            continue

        timestamp = datetime.datetime.fromtimestamp(float(rows[0][0])).strftime("%Y-%m-%d-%H-%M-%S")
        frames = []
        joint_history = {
            robot: np.asarray([row[1 + len(camera_names) + robot_idx] for row in rows], dtype=np.float32)
            for robot_idx, robot in enumerate(robot_names)
        }
        gripper_history = {
            robot: np.asarray(
                [row[1 + len(camera_names) + len(robot_names) + robot_idx] for row in rows], dtype=np.float32
            )
            for robot_idx, robot in enumerate(robot_names)
        }
        cols = math.ceil(math.sqrt(len(camera_names) + 1))
        rows_per_frame = math.ceil((len(camera_names) + 1) / cols)

        for step_idx, row in enumerate(rows):
            decoded = [
                decode_jpeg(torch.frombuffer(bytearray(image_bytes), dtype=torch.uint8)).permute(1, 2, 0).cpu().numpy()
                for image_bytes in row[1 : 1 + len(camera_names)]
            ]
            height, width = decoded[0].shape[:2]
            decoded.append(_render_action_panel(joint_history, gripper_history, step_idx, height, width))
            tiled = np.zeros((rows_per_frame * height, cols * width, 3), dtype=np.uint8)
            for camera_index, image in enumerate(decoded):
                top = (camera_index // cols) * height
                left = (camera_index % cols) * width
                tiled[top : top + height, left : left + width] = image
            frames.append(tiled)

        _write_mp4(frames, output / f"{timestamp}.mp4", fps=fps)
