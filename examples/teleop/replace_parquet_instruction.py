from __future__ import annotations

import argparse
import os
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq


def _instruction_field(schema: pa.Schema) -> pa.Field:
    # Fail loudly if the dataset doesn't have the column we're about to replace.
    if "instruction" not in schema.names:
        raise ValueError("Parquet file has no 'instruction' column")
    return schema.field("instruction")


def overwrite_file(source_file: Path, instruction: str, compression: str) -> None:
    """Rewrite ``source_file`` in place with ``instruction`` in every row."""
    reader = pq.ParquetFile(source_file)
    field = _instruction_field(reader.schema_arrow)  # keep the column's exact Arrow type

    # Write to a sibling temp file first; we swap it in only once fully written.
    tmp_file = source_file.with_suffix(source_file.suffix + ".tmp")
    if tmp_file.exists():
        raise FileExistsError(f"Refusing to overwrite existing temp file: {tmp_file}")

    writer = pq.ParquetWriter(tmp_file, reader.schema_arrow, compression=compression)
    try:
        column_index = reader.schema_arrow.names.index("instruction")
        # Stream one row group at a time so the ~7 GB file never loads fully into RAM.
        for group_index in range(reader.num_row_groups):
            batch = reader.read_row_group(group_index)
            instructions = pa.array([instruction] * batch.num_rows, type=field.type)
            batch = batch.set_column(column_index, field, instructions)
            writer.write_table(batch)
        writer.close()
    except BaseException:
        # On any failure, drop the partial temp file and leave the original untouched.
        writer.close()
        tmp_file.unlink(missing_ok=True)
        raise

    os.replace(tmp_file, source_file)  # atomic swap: original is only ever whole
    print(f"Overwrote instruction in {source_file}")


def overwrite_dataset(folder: Path, instruction: str, compression: str) -> None:
    folder = folder.resolve()
    if not folder.is_dir():
        raise ValueError(f"Path must be a folder containing .parquet file(s): {folder}")
    # Handle every .parquet under the folder (datasets may be split into parts).
    files = sorted(folder.rglob("*.parquet"))
    if not files:
        raise ValueError(f"No .parquet files found under {folder}")
    for source_file in files:
        overwrite_file(source_file, instruction, compression)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", type=Path, help="Folder containing the .parquet file(s) to rewrite in place")
    parser.add_argument("--instruction", required=True, help="Replacement instruction to store in every row")
    parser.add_argument("--compression", default="zstd", help="Parquet compression codec (default: zstd)")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    overwrite_dataset(args.folder, args.instruction, args.compression)
  