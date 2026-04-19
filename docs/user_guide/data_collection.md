# Data Collection

RCS provides tools for efficiently collecting and managing robot interaction data, primarily for Imitation Learning and Reinforcement Learning.

## StorageWrapper

The `StorageWrapper` is the primary tool for recording environment transitions. It is designed to be crash-safe and efficient.

### Key Features

- **Asynchronous Writing**: Data is written to disk in a background thread to minimize impact on the control loop.
- **Crash-Safe**: Data is flushed in atomic batches, ensuring that most data is preserved even if the process crashes.
- **Parquet Format**: Uses the Apache Parquet format (via `pyarrow`) for efficient storage and fast reading.
- **Automatic Consolidation**: Small batch files are automatically merged into larger optimized files when the environment is closed.
- **Image Compression**: RGB frames are automatically encoded as JPEGs to save space.

### Usage

```python
from rcs.envs.storage_wrapper import StorageWrapper

# Wrap your environment
env = StorageWrapper(
    env,
    base_dir="data/my_experiment",
    instruction="pick up the red cube",
    always_record=False  # Only record when start_record() is called
)

# Control recording
env.start_record()
# ... perform tasks ...
env.stop_record()

# Close to ensure all data is flushed and consolidated
env.close()
```

## Dataset Structure

The data is organized in a directory structure partitioned by date:

```text
base_dir/
  date=2024-05-20/
    part-0-a1b2c3d4.parquet
    part-1-e5f6g7h8.parquet
  date=2024-05-21/
    ...
```

Each Parquet file contains:
- `obs`: The environment observation (flattened).
- `action`: The action taken.
- `reward`: The reward received.
- `success`: Boolean indicating task success.
- `instruction`: The text instruction for the task.
- `timestamp`: Unix timestamp of the transition.
- `uuid`: A unique identifier for the episode.

## Consolidating Data

If a script exits unexpectedly and consolidation doesn't run, you can manually consolidate the fragmented files using the `StorageWrapper.consolidate` static method or the RCS CLI:

```shell
python -m rcs consolidate data/my_experiment
```
