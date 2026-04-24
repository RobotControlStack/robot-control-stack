import logging

import cv2
import typer
from rcs_usb_cam.camera import USBCameraConfig, USBCameraSet

logger = logging.getLogger(__name__)
usb_cam_app = typer.Typer(help="CLI tools for the generic USB camera module of rcs.")


def _capture_identifier(identifier: str) -> str | int:
    return int(identifier) if identifier.isdigit() else identifier


def _display_frame(window_name: str, frame):
    cv2.imshow(window_name, frame.camera.color.data)


@usb_cam_app.command("rgb-view")
def rgb_view(
    identifier: str = typer.Argument("/dev/video0", help="Video device path or numeric camera id."),
    width: int = typer.Option(640, help="Requested capture width."),
    height: int = typer.Option(480, help="Requested capture height."),
    fps: int = typer.Option(30, help="Requested capture frame rate."),
    window_name: str = typer.Option("USB Camera RGB", help="OpenCV window title."),
):
    """Open a live RGB window using the RCS USB camera interface."""
    camera_identifier = _capture_identifier(identifier)
    camera = USBCameraSet(
        cameras={
            "viewer": USBCameraConfig(
                identifier=camera_identifier,  # type: ignore[arg-type]
                resolution_width=width,
                resolution_height=height,
                frame_rate=fps,
            )
        }
    )

    try:
        camera.open()
    except Exception as exc:
        msg = f"Could not open USB camera {identifier}: {exc}"
        raise typer.BadParameter(msg) from exc

    logger.info("Streaming RGB from %s. Press 'q' to quit.", identifier)
    try:
        while True:
            frame = camera.poll_frame("viewer")
            _display_frame(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.close()
        cv2.destroyAllWindows()


def main():
    usb_cam_app()


if __name__ == "__main__":
    main()
