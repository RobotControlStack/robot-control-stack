#!/usr/bin/env python3
"""Add wrist-frame Robotiq 2F-85 finger poses to a parquet dataset.

The input observations are expected to have ``obs.<arm>.gripper``, where the
gripper state is normalized from 0 (closed) to 1 (open).  The output adds
``obs.<arm>.fingers`` with this schema::

    fingers: {
        left:  [[...], [...], [...], [...]],
        right: [[...], [...], [...], [...]],
    }

Each value is a 4x4 homogeneous transform in the gripper-base (wrist) frame.
The script reads and writes in batches, so it can be used with large datasets.

Example:
    python examples/teleop/add_robotiq_finger_poses_to_parquet.py \
        /home/bien/Documents/Development/RCS/datasets/dataset_parquet/utn_usbc_insertion_with_absolute_action \
        --output /home/bien/Documents/Development/RCS/datasets/dataset_parquet/utn_usbc_insertion_with_absolute_action_with_fingers
"""

from __future__ import annotations

import argparse
import shutil
import tempfile
from pathlib import Path
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from rcs_robotiq2f85.kinematics import ROBOTIQ_2F85_CACHE_STEP, robotiq_2f85_finger_poses


DEFAULT_LEFT_SITE = "left_digit_pad"
DEFAULT_RIGHT_SITE = "right_digit_pad"


def _matrix_array(matrices: np.ndarray) -> pa.FixedSizeListArray:
    """Convert ``(N, 4, 4)`` matrices into an Arrow fixed-size 4x4 list."""
    values = pa.array(matrices.reshape(-1), type=pa.float64())
    rows = pa.FixedSizeListArray.from_arrays(values, 4)
    return pa.FixedSizeListArray.from_arrays(rows, 4)


def _finger_pose_lookup(
    *,
    left_site: str,
    right_site: str,
    model_path: Path | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the 0.001-resolution pose lookup once through RCS kinematics."""
    states = np.arange(1001, dtype=np.float64) * ROBOTIQ_2F85_CACHE_STEP
    pairs = [
        robotiq_2f85_finger_poses(
            float(state),
            site_name=(left_site, right_site),
            model_path=model_path,
        )
        for state in states
    ]
    return (
        np.stack([pair.left.pose_matrix() for pair in pairs]),
        np.stack([pair.right.pose_matrix() for pair in pairs]),
    )


def _normalized_gripper_values(gripper: pa.Array) -> np.ndarray:
    """Return scalar gripper values, with a clear error for malformed rows."""
    values = []
    for index in range(len(gripper)):
        value = gripper[index].as_py()
        if value is None:
            msg = f"gripper value at batch offset {index} is null"
            raise ValueError(msg)
        array = np.asarray(value, dtype=np.float64)
        if array.size != 1:
            msg = f"gripper value at batch offset {index} must contain one value, got shape {array.shape}"
            raise ValueError(msg)
        values.append(float(array.item()))

    result = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(result)) or np.any(result < 0.0) or np.any(result > 1.0):
        msg = "gripper values must be finite and in the normalized range [0, 1]"
        raise ValueError(msg)
    return result


def _replace_arm_with_fingers(
    table: pa.Table,
    *,
    arm: str,
    left_lookup: np.ndarray,
    right_lookup: np.ndarray,
) -> pa.Table:
    """Return a batch with ``obs.<arm>.fingers`` appended to its struct."""
    obs_index = table.schema.get_field_index("obs")
    if obs_index == -1:
        raise KeyError("Input parquet must contain an 'obs' struct")

    obs = table.column(obs_index).combine_chunks()
    if not isinstance(obs, pa.StructArray) or obs.type.get_field_index(arm) == -1:
        raise KeyError(f"Input parquet must contain an 'obs.{arm}' struct")
    arm_obs = obs.field(arm)
    if not isinstance(arm_obs, pa.StructArray) or arm_obs.type.get_field_index("gripper") == -1:
        raise KeyError(f"Input parquet must contain an 'obs.{arm}.gripper' field")
    if arm_obs.type.get_field_index("fingers") != -1:
        raise ValueError(f"Input parquet already contains 'obs.{arm}.fingers'")

    gripper = _normalized_gripper_values(arm_obs.field("gripper"))
    indices = np.rint(gripper / ROBOTIQ_2F85_CACHE_STEP).astype(np.intp)
    fingers = pa.StructArray.from_arrays(
        [_matrix_array(left_lookup[indices]), _matrix_array(right_lookup[indices])],
        names=["left", "right"],
    )
    new_arm = pa.StructArray.from_arrays(
        [arm_obs.field(name) for name in arm_obs.type.names] + [fingers],
        names=[*arm_obs.type.names, "fingers"],
    )
    new_obs = pa.StructArray.from_arrays(
        [new_arm if name == arm else obs.field(name) for name in obs.type.names],
        names=obs.type.names,
    )
    return table.set_column(obs_index, "obs", new_obs)


def _input_files(input_path: Path) -> list[Path]:
    if input_path.is_file():
        if input_path.suffix != ".parquet":
            raise ValueError(f"Input file must end in .parquet: {input_path}")
        return [input_path]
    if input_path.is_dir():
        files = sorted(input_path.rglob("*.parquet"))
        if files:
            return files
        raise ValueError(f"No parquet files found below {input_path}")
    raise FileNotFoundError(input_path)


def _output_path(input_path: Path) -> Path:
    suffix = "_with_fingers"
    if input_path.is_file():
        return input_path.with_stem(input_path.stem + suffix)
    return input_path.with_name(input_path.name + suffix)


def _write_file(
    source: Path,
    destination: Path,
    *,
    arm: str,
    batch_size: int,
    compression: str,
    left_lookup: np.ndarray,
    right_lookup: np.ndarray,
) -> int:
    parquet_file = pq.ParquetFile(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    row_count = 0
    writer: pq.ParquetWriter | None = None
    try:
        for batch in parquet_file.iter_batches(batch_size=batch_size):
            table = _replace_arm_with_fingers(
                pa.Table.from_batches([batch]), arm=arm, left_lookup=left_lookup, right_lookup=right_lookup
            )
            if writer is None:
                writer = pq.ParquetWriter(destination, table.schema, compression=compression)
            writer.write_table(table)
            row_count += table.num_rows
    finally:
        if writer is not None:
            writer.close()
    return row_count


def _destination_for(source: Path, input_path: Path, output_path: Path) -> Path:
    if input_path.is_file():
        return output_path
    return output_path / source.relative_to(input_path)


def add_finger_poses(
    input_path: Path,
    output_path: Path,
    *,
    arm: str,
    left_site: str,
    right_site: str,
    model_path: Path | None,
    batch_size: int,
    compression: str,
) -> None:
    """Write a copy of ``input_path`` whose selected arm has finger poses."""
    input_path = input_path.resolve()
    output_path = output_path.resolve()
    if output_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output_path}")
    if input_path == output_path or input_path in output_path.parents:
        raise ValueError("Output must not be inside the input dataset")

    left_lookup, right_lookup = _finger_pose_lookup(
        left_site=left_site,
        right_site=right_site,
        model_path=model_path,
    )
    sources = _input_files(input_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_root = Path(tempfile.mkdtemp(prefix=f".{output_path.name}-", dir=output_path.parent))
    temporary_output = temporary_root / output_path.name
    try:
        total_rows = 0
        for source in sources:
            destination = _destination_for(source, input_path, temporary_output)
            rows = _write_file(
                source,
                destination,
                arm=arm,
                batch_size=batch_size,
                compression=compression,
                left_lookup=left_lookup,
                right_lookup=right_lookup,
            )
            total_rows += rows
            print(f"Wrote {rows:,} rows to {destination}")
        temporary_output.replace(output_path)
        print(f"Added obs.{arm}.fingers to {total_rows:,} rows in {output_path}")
    except Exception:
        shutil.rmtree(temporary_root, ignore_errors=True)
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Input parquet file or directory of parquet files.")
    parser.add_argument("--output", type=Path, help="New output path (never overwrites an existing path).")
    parser.add_argument("--arm", default="right", help="Arm below obs that owns gripper and fingers (default: right).")
    parser.add_argument("--left-site", default=DEFAULT_LEFT_SITE, help=f"Left finger site (default: {DEFAULT_LEFT_SITE}).")
    parser.add_argument("--right-site", default=DEFAULT_RIGHT_SITE, help=f"Right finger site (default: {DEFAULT_RIGHT_SITE}).")
    parser.add_argument("--model-path", type=Path, help="Optional Robotiq MJCF model; defaults to RCS's 2F-85 model.")
    parser.add_argument("--batch-size", type=int, default=16_384, help="Rows per read/write batch (default: 16384).")
    parser.add_argument("--compression", default="zstd", help="Parquet compression codec (default: zstd).")
    args = parser.parse_args()
    if args.batch_size <= 0:
        parser.error("--batch-size must be positive")
    return args


def main() -> None:
    args = parse_args()
    add_finger_poses(
        args.input,
        args.output or _output_path(args.input),
        arm=args.arm,
        left_site=args.left_site,
        right_site=args.right_site,
        model_path=args.model_path,
        batch_size=args.batch_size,
        compression=args.compression,
    )


if __name__ == "__main__":
    main()
