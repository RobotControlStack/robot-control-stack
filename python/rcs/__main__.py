from pathlib import Path
from typing import Annotated

import typer
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.lerobot_joint_converter import (
    DEFAULT_BINARIZE_GRIPPER,
    DEFAULT_CAMERAS,
    DEFAULT_DATASET_PATHS,
    DEFAULT_FPS,
    DEFAULT_GRIPPER_BINARIZE_THRESHOLD,
    DEFAULT_GRIPPER_TYPE,
    DEFAULT_HF_DATA_DIR,
    DEFAULT_IMAGE_BATCH_SIZE,
    DEFAULT_JOINTS,
    DEFAULT_PER_ROBOT_ARM_DIM,
    DEFAULT_REPO_ID,
    DEFAULT_ROBOT_KEYS,
    DEFAULT_ROBOT_TYPE,
    camera_specs_to_configs,
    run_conversion,
)
from rcs.sim.replayer import replay as replay_dataset

app = typer.Typer()


def _exec_import_statements(imports: list[str] | None) -> None:
    if imports is None:
        return

    for import_statement in imports:
        exec(import_statement, {})


@app.command()
def consolidate(
    path: Annotated[
        Path,
        typer.Argument(
            exists=True,
            file_okay=False,
            dir_okay=True,
            help="The root directory of the parquet dataset to consolidate.",
        ),
    ]
):
    """
    Consolidates a fragmented Parquet dataset into larger files.

    This is useful if the recording process crashed or was interrupted,
    leaving many small files behind.
    """
    typer.echo(f"Starting consolidation for: {path}")

    StorageWrapper.consolidate(str(path), schema=None)

    typer.echo("Done.")


@app.command("replay")
def replay(
    dataset: Annotated[
        Path,
        typer.Argument(
            exists=True,
            help="Parquet dataset directory to replay.",
        ),
    ],
    output: Annotated[
        Path,
        typer.Argument(
            exists=False,
            help="Output dir for the new dataset.",
        ),
    ],
    headless: Annotated[bool, typer.Option(help="Whether to run without GUI.")] = True,
    frequency: Annotated[int, typer.Option(help="Simulation frequency to use during replay.")] = 30,
    relative_to: Annotated[
        str,
        typer.Option(help="RelativeTo enum name: CONFIGURED_ORIGIN, LAST_STEP, or NONE."),
    ] = "CONFIGURED_ORIGIN",
    env_id: Annotated[
        str,
        typer.Option(help="Environment id used in gym.make()."),
    ] = "rcs/duo",
    imports: Annotated[
        list[str] | None,
        typer.Option(
            "--import",
            help="Python import statement to execute before resolving the environment. Repeat for multiple imports. Example: --import 'from rcs_duobench.tasks import bin_sort'",
        ),
    ] = None,
):
    _exec_import_statements(imports)
    replay_dataset(
        dataset=dataset,
        output=output,
        headless=headless,
        frequency=frequency,
        relative_to=relative_to,
        env_id=env_id,
    )


@app.command("lerobot-convert")
def lerobot_convert(
    output: Annotated[
        Path,
        typer.Argument(
            help="Output directory for the LeRobot dataset. Example: ./data_lerobot",
        ),
    ] = Path(DEFAULT_HF_DATA_DIR),
    dataset_paths: Annotated[
        list[str] | None,
        typer.Option(
            "--dataset-path",
            help="Input parquet path or glob. Repeat for multiple datasets. Example: --dataset-path /data/session1 --dataset-path /data/session2",
        ),
    ] = None,
    repo_id: Annotated[
        str, typer.Option(help="LeRobot repo id metadata. Example: --repo-id myorg/grasp_v2")
    ] = DEFAULT_REPO_ID,
    robot_type: Annotated[
        str, typer.Option(help="Robot type for metadata and IK model lookup. Example: --robot-type FR3")
    ] = DEFAULT_ROBOT_TYPE,
    fps: Annotated[int, typer.Option(help="Dataset frames per second. Example: --fps 30")] = DEFAULT_FPS,
    robot_keys: Annotated[
        list[str] | None,
        typer.Option(
            "--robot-key",
            help="Robot keys to concatenate. Repeat for multiple robots. Example: --robot-key left --robot-key right",
        ),
    ] = None,
    joints: Annotated[
        bool, typer.Option(help="Whether absolute_action is already in joint space. Example: --joints")
    ] = DEFAULT_JOINTS,
    gripper_type: Annotated[
        str, typer.Option(help="Gripper type used to derive TCP offset. Example: --gripper-type Robotiq2F85")
    ] = DEFAULT_GRIPPER_TYPE,
    camera_specs: Annotated[
        list[str] | None,
        typer.Option(
            "--camera",
            help=(
                "Camera spec as name[:source_name][@HEIGHTxWIDTH]. Repeat for multiple cameras. "
                "The name becomes the LeRobot output key (observation.images.<name>). "
                "The optional source_name is the key in the source parquet (obs.frames.<source_name>.rgb.data); "
                "if omitted, the image_ prefix is stripped from name to derive it. "
                "Example: --camera head@256x256 --camera image_left_wrist:left_wrist@256x256"
            ),
        ),
    ] = None,
    image_batch_size: Annotated[
        int, typer.Option(help="Batch size for image decoding. Example: --image-batch-size 32")
    ] = DEFAULT_IMAGE_BATCH_SIZE,
    per_robot_arm_dim: Annotated[
        int, typer.Option(help="Per-robot arm joint/action dimension without gripper. Example: --per-robot-arm-dim 7")
    ] = DEFAULT_PER_ROBOT_ARM_DIM,
    binarize_gripper: Annotated[
        bool, typer.Option(help="Binarize gripper values before export. Example: --binarize-gripper")
    ] = DEFAULT_BINARIZE_GRIPPER,
    gripper_binarize_threshold: Annotated[
        float,
        typer.Option(
            help="Threshold used when binarizing gripper values; values above this become 1.0. Example: --gripper-binarize-threshold 0.2"
        ),
    ] = DEFAULT_GRIPPER_BINARIZE_THRESHOLD,
    success: Annotated[bool, typer.Option(help="Only include successful episodes. Example: --success")] = True,
    n: Annotated[int, typer.Option(help="Maximum number of episodes to convert. -1 means all. Example: --n 50")] = -1,
    video_encoding: Annotated[bool, typer.Option(help="Should the image data be video encoded")] = False,
    video_backend: Annotated[
        str | None, typer.Option(help="Video backend to use if image data is video encoded e.g. torchcodec")
    ] = None,
):
    cameras = camera_specs_to_configs(camera_specs) if camera_specs is not None else list(DEFAULT_CAMERAS)
    run_conversion(
        root=output,
        dataset_paths=dataset_paths or list(DEFAULT_DATASET_PATHS),
        repo_id=repo_id,
        robot_type=robot_type,
        fps=fps,
        robot_keys=robot_keys or list(DEFAULT_ROBOT_KEYS),
        joints=joints,
        gripper_type=gripper_type,
        cameras=cameras,
        image_batch_size=image_batch_size,
        per_robot_arm_dim=per_robot_arm_dim,
        binarize_gripper=binarize_gripper,
        gripper_binarize_threshold=gripper_binarize_threshold,
        success=success,
        n=n,
        video_encoding=video_encoding,
        video_backend=video_backend,
    )


if __name__ == "__main__":
    app()
