#!/usr/bin/env python3

from pathlib import Path

import duckdb
import numpy as np


PARQUET_PATH = Path(
    "/home/bien/Documents/Development/RCS/robot-control-stack/"
    "examples/teleop/rumi_pnp_new/2026-09-07_part-0.parquet"
)


def average_episode_start_joints(parquet_path: Path) -> np.ndarray:
    query = """
        WITH first_frames AS (
            SELECT
                uuid,
                step,
                obs.right.joints AS joints,
                ROW_NUMBER() OVER (
                    PARTITION BY uuid
                    ORDER BY step
                ) AS frame_rank
            FROM read_parquet(?)
            WHERE uuid IS NOT NULL
        )
        SELECT uuid, step, joints
        FROM first_frames
        WHERE frame_rank = 1
        ORDER BY uuid
    """

    with duckdb.connect() as connection:
        rows = connection.execute(query, [str(parquet_path)]).fetchall()

    if not rows:
        raise ValueError("No episodes were found.")

    episode_starts = []

    for uuid, step, joints in rows:
        joints = np.asarray(joints, dtype=np.float64).reshape(-1)

        if joints.shape != (7,):
            raise ValueError(
                f"Episode {uuid} has unexpected joint shape {joints.shape}"
            )

        if not np.isfinite(joints).all():
            raise ValueError(f"Episode {uuid} contains non-finite joint values")

        episode_starts.append(joints)

        print(
            f"Episode {uuid}, first step {step}: "
            f"{np.array2string(joints, precision=8, separator=', ')}"
        )

    episode_starts = np.stack(episode_starts)
    average_joints = np.mean(episode_starts, axis=0)
    standard_deviation = np.std(episode_starts, axis=0)

    print(f"\nEpisodes: {len(episode_starts)}")
    print(
        "Average q_home [rad]: "
        f"{np.array2string(average_joints, precision=8, separator=', ')}"
    )
    print(
        "Start-pose standard deviation [rad]: "
        f"{np.array2string(standard_deviation, precision=8, separator=', ')}"
    )

    return average_joints


if __name__ == "__main__":
    average_episode_start_joints(PARQUET_PATH)