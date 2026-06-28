import logging
from pathlib import Path
from time import monotonic

import cv2
import typer
from rcs_zed.camera import ZEDCameraSet

from rcs import common

logger = logging.getLogger(__name__)
zed_app = typer.Typer(help="CLI tools for the ZED camera module of rcs.")


def _display_frame(window_name: str, frame):
    cv2.imshow(window_name, frame.camera.color.data[:, :, ::-1])


def _resolve_serial(serial: str | None) -> str:
    if serial is not None:
        return serial

    try:
        devices = ZEDCameraSet.enumerate_connected_devices()
    except RuntimeError as exc:
        msg = str(exc)
        raise typer.BadParameter(msg) from exc
    if len(devices) == 0:
        msg = "No ZED devices connected."
        raise typer.BadParameter(msg)
    return next(iter(devices))


def _create_rgb_camera(serial: str, width: int, height: int, fps: int) -> ZEDCameraSet:
    return ZEDCameraSet(
        cameras={
            "viewer": common.BaseCameraConfig(
                identifier=serial,
                resolution_width=width,
                resolution_height=height,
                frame_rate=fps,
            )
        },
        enable_depth=False,
        enable_imu=False,
    )


@zed_app.command()
def serials():
    """Reads out the serial numbers and models of connected ZED devices via the SDK."""
    try:
        devices = ZEDCameraSet.enumerate_connected_devices()
    except RuntimeError as exc:
        typer.secho(str(exc), fg=typer.colors.YELLOW, err=True)
        return
    if len(devices) == 0:
        typer.secho("No ZED devices connected or the ZED SDK is not available.", fg=typer.colors.YELLOW, err=True)
        return
    typer.echo("Connected devices:")
    for device in devices.values():
        typer.echo(f"  {device.model}: {device.serial} (imu={device.has_imu})")


@zed_app.command("rgb-view")
def rgb_view(
    serial: str | None = typer.Argument(None, help="Optional ZED serial number. Uses the first device if omitted."),
    width: int = typer.Option(1280, help="Requested capture width."),
    height: int = typer.Option(720, help="Requested capture height."),
    fps: int = typer.Option(30, help="Requested capture frame rate."),
    window_name: str = typer.Option("ZED RGB", help="OpenCV window title."),
):
    """Open a live RGB window using the RCS ZED camera interface."""
    serial = _resolve_serial(serial)
    camera = _create_rgb_camera(serial, width, height, fps)

    try:
        camera.open()
    except Exception as exc:
        msg = f"Could not start ZED camera {serial}: {exc}"
        raise typer.BadParameter(msg) from exc

    logger.info("Streaming RGB from ZED %s. Press 'q' to quit.", serial)
    try:
        while True:
            frame = camera.poll_frame("viewer")
            _display_frame(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.close()
        cv2.destroyAllWindows()


@zed_app.command("rgb-snapshot")
def rgb_snapshot(
    serial: str | None = typer.Argument(None, help="Optional ZED serial number. Uses the first device if omitted."),
    output: str = typer.Option("zed_latest.png", "--output", "-o", help="PNG file to write."),
    width: int = typer.Option(1280, help="Requested capture width."),
    height: int = typer.Option(720, help="Requested capture height."),
    fps: int = typer.Option(30, help="Requested capture frame rate."),
    duration: float = typer.Option(1.0, "--duration", "-d", help="Seconds to read frames before saving."),
):
    """Open a ZED camera briefly and save the latest RGB frame as a PNG."""
    output_path = Path(output)
    if duration <= 0:
        msg = "Duration must be greater than zero."
        raise typer.BadParameter(msg)
    if output_path.suffix.lower() != ".png":
        msg = "Output path must end in .png."
        raise typer.BadParameter(msg)

    serial = _resolve_serial(serial)
    camera = _create_rgb_camera(serial, width, height, fps)

    try:
        camera.open()
    except Exception as exc:
        msg = f"Could not start ZED camera {serial}: {exc}"
        raise typer.BadParameter(msg) from exc

    latest_frame = None
    frames_read = 0
    deadline = monotonic() + duration
    try:
        while latest_frame is None or monotonic() < deadline:
            latest_frame = camera.poll_frame("viewer")
            frames_read += 1
    finally:
        camera.close()

    assert latest_frame is not None
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image_bgr = latest_frame.camera.color.data[:, :, ::-1]
    print(latest_frame.camera.color.intrinsics)
    if not cv2.imwrite(str(output_path), image_bgr):
        msg = f"Could not write PNG to {output_path}."
        raise typer.BadParameter(msg)

    typer.echo(f"Saved {output} from ZED {serial} after reading {frames_read} frame(s).")


def main():
    zed_app()


if __name__ == "__main__":
    main()
