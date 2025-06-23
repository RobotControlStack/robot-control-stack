import logging

import pyrealsense2 as rs
import typer
from rcs_realsense.camera import RealSenseCameraSet

logger = logging.getLogger(__name__)
realsense_app = typer.Typer(help="CLI tool for the intel realsense module of rcs.")


@realsense_app.command()
def serials():
    """Reads out the serial numbers of the connected realsense devices."""
    context = rs.context()
    devices = RealSenseCameraSet.enumerate_connected_devices(context)
    if len(devices) == 0:
        logger.warning("No realsense devices connected.")
        return
    logger.info("Connected devices:")
    for device in devices.values():
        logger.info("  %s: %s", device.product_line, device.serial)


if __name__ == "__main__":
    realsense_app()
