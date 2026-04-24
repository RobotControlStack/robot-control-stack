import logging

import cv2
import pyrealsense2 as rs
import typer
from rcs_realsense.camera import RealSenseCameraSet

from rcs import common

logger = logging.getLogger(__name__)
realsense_app = typer.Typer(help="CLI tool for the intel realsense module of rcs.")


def _display_frame(window_name: str, frame, *, is_rgb: bool):
    image = frame.camera.color.data
    if is_rgb:
        image = image[:, :, ::-1]
    cv2.imshow(window_name, image)


@realsense_app.command()
def serials():
    """Reads out the serial numbers of the connected realsense devices."""
    try:
        context = rs.context()
        devices = RealSenseCameraSet.enumerate_connected_devices(context)
    except Exception as exc:
        typer.secho(f"Could not enumerate RealSense devices: {exc}", fg=typer.colors.RED, err=True)
        return

    if len(devices) == 0:
        typer.secho("No RealSense devices connected.", fg=typer.colors.YELLOW, err=True)
        return

    typer.echo("Connected devices:")
    for device in devices.values():
        typer.echo(f"  {device.product_line}: {device.serial}")


@realsense_app.command("rgb-view")
def rgb_view(
    serial: str | None = typer.Argument(
        None, help="Optional RealSense serial number. Uses the first device if omitted."
    ),
    width: int = typer.Option(1280, help="Requested capture width."),
    height: int = typer.Option(720, help="Requested capture height."),
    fps: int = typer.Option(30, help="Requested capture frame rate."),
    window_name: str = typer.Option("RealSense RGB", help="OpenCV window title."),
):
    """Open a live RGB window using the RCS RealSense camera interface."""
    if serial is None:
        devices = RealSenseCameraSet.enumerate_connected_devices(rs.context())
        if len(devices) == 0:
            msg = "No RealSense devices connected."
            raise typer.BadParameter(msg)
        serial = next(iter(devices))

    camera = RealSenseCameraSet(
        cameras={
            "viewer": common.BaseCameraConfig(
                identifier=serial,
                resolution_width=width,
                resolution_height=height,
                frame_rate=fps,
            )
        },
        enable_ir=False,
        enable_imu=False,
    )

    try:
        camera.open()
    except Exception as exc:
        msg = f"Could not start RealSense camera {serial}: {exc}"
        raise typer.BadParameter(msg) from exc

    logger.info("Streaming RGB from RealSense %s. Press 'q' to quit.", serial)
    try:
        while True:
            frame = camera.poll_frame("viewer")
            _display_frame(window_name, frame, is_rgb=True)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    realsense_app()
