#!/usr/bin/env python3
"""Add joint-space ``absolute_action`` fields to bilateral teleop Parquet files.

The bilateral wrapper records the leader target in
``info.<robot_key>.bilateral_leader_action.joints`` instead of the standard
``info.<robot_key>.absolute_action`` field expected by the LeRobot converter.
This utility writes a new dataset directory with the latter field populated
from the former. It can also replace the instruction stored in every row. It
never changes the input directory.

Example:
    python examples/teleop/rewrite_bilateral_absolute_action.py \
        examples/teleop/utn_usbc_insertion \
        examples/teleop/utn_usbc_insertion_with_absolute_action \
        --instruction "insert the USB-C connector"
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq


def add_absolute_action(table: pa.Table, robot_key: str, instruction: str | None) -> pa.Table:
    """Add the bilateral joint target and, optionally, replace the instruction."""
    if "info" not in table.column_names:
        raise ValueError("Parquet file has no 'info' column")

    info = table["info"].combine_chunks()
    if not isinstance(info, pa.StructArray):
        raise ValueError("The 'info' column is not a struct")
    if robot_key not in info.type.names:
        raise ValueError(f"The 'info' struct has no '{robot_key}' field")

    robot = info.field(robot_key)
    if not isinstance(robot, pa.StructArray):
        raise ValueError(f"The 'info.{robot_key}' field is not a struct")
    if "bilateral_leader_action" not in robot.type.names:
        raise ValueError(f"The 'info.{robot_key}' struct has no 'bilateral_leader_action' field")

    leader_action = robot.field("bilateral_leader_action")
    if not isinstance(leader_action, pa.StructArray) or "joints" not in leader_action.type.names:
        raise ValueError(f"The 'info.{robot_key}.bilateral_leader_action.joints' field is missing")
    joints = leader_action.field("joints")

    robot_fields = list(robot.type)
    robot_arrays = [robot.field(field.name) for field in robot_fields]
    if "absolute_action" in robot.type.names:
        index = robot.type.get_field_index("absolute_action")
        robot_arrays[index] = joints
        robot_fields[index] = pa.field("absolute_action", joints.type, nullable=True)
    else:
        robot_fields.append(pa.field("absolute_action", joints.type, nullable=True))
        robot_arrays.append(joints)
    rewritten_robot = pa.StructArray.from_arrays(robot_arrays, fields=robot_fields, mask=robot.is_null())

    info_fields = list(info.type)
    info_arrays = [info.field(field.name) for field in info_fields]
    robot_index = info.type.get_field_index(robot_key)
    info_arrays[robot_index] = rewritten_robot
    info_fields[robot_index] = pa.field(robot_key, rewritten_robot.type, nullable=True)
    rewritten_info = pa.StructArray.from_arrays(info_arrays, fields=info_fields, mask=info.is_null())

    info_index = table.column_names.index("info")
    rewritten_table = table.set_column(
        info_index, pa.field("info", rewritten_info.type, nullable=True), rewritten_info
    )
    if instruction is None:
        return rewritten_table
    if "instruction" not in rewritten_table.column_names:
        raise ValueError("Parquet file has no 'instruction' column")

    instruction_index = rewritten_table.column_names.index("instruction")
    instruction_field = rewritten_table.schema.field(instruction_index)
    instructions = pa.array([instruction] * rewritten_table.num_rows, type=instruction_field.type)
    return rewritten_table.set_column(instruction_index, instruction_field, instructions)


def rewrite_dataset(
    source: Path, destination: Path, robot_key: str, compression: str, instruction: str | None
) -> None:
    source = source.resolve()
    destination = destination.resolve()
    if not source.is_dir():
        raise ValueError(f"Source must be a dataset directory: {source}")
    if source == destination:
        raise ValueError("Destination must differ from source; this script does not modify source files.")
    files = sorted(source.rglob("*.parquet"))
    if not files:
        raise ValueError(f"No Parquet files found under {source}")

    for source_file in files:
        destination_file = destination / source_file.relative_to(source)
        if destination_file.exists():
            raise FileExistsError(f"Refusing to overwrite existing file: {destination_file}")
        destination_file.parent.mkdir(parents=True, exist_ok=True)
        rewritten = add_absolute_action(pq.read_table(source_file), robot_key, instruction)
        pq.write_table(rewritten, destination_file, compression=compression)
        print(f"Wrote {destination_file}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Existing bilateral teleop dataset directory")
    parser.add_argument("destination", type=Path, help="New directory for converted Parquet files")
    parser.add_argument("--robot-key", default="right", help="Robot key below info (default: right)")
    parser.add_argument("--compression", default="zstd", help="Parquet compression codec (default: zstd)")
    parser.add_argument(
        "--instruction",
        help="Replacement instruction to store in every row; omit to retain the recorded instruction",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rewrite_dataset(args.source, args.destination, args.robot_key, args.compression, args.instruction)
