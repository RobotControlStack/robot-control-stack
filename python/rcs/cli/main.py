import logging
from contextlib import ExitStack
from time import sleep
from typing import Annotated, Optional

import pyrealsense2 as rs
import rcs
import rcs.control.fr3_desk
import typer
from rcs.camera.realsense import RealSenseCameraSet
from rcs.control.fr3_desk import load_creds_fr3_desk
from rcs.envs.creators import get_urdf_path

logger = logging.getLogger(__name__)

# MAIN CLI
main_app = typer.Typer(help="CLI tool for the Robot Control Stack (RCS).")


# REALSENSE CLI
realsense_app = typer.Typer()
main_app.add_typer(
    realsense_app,
    name="realsense",
    help="Commands to access the intel realsense camera. This includes tools such as reading out the serial numbers of connected devices.",
)


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


# FR3 CLI
fr3_app = typer.Typer()
main_app.add_typer(
    fr3_app,
    name="fr3",
    help="Commands to control a Franka Research 3. This includes tools that you would usually do with Franka's Desk interface.",
)


@fr3_app.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    shut: Annotated[bool, typer.Option("-s", help="Should the robot be shut down")] = False,
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
):
    """Moves the FR3 to home position"""
    user, pw = load_creds_fr3_desk()
    rcs.control.fr3_desk.home(ip, user, pw, shut, unlock)


@fr3_app.command()
def info(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    include_gripper: Annotated[bool, typer.Option("-g", help="includes gripper")] = False,
):
    """Prints info about the robots current joint position and end effector pose, optionally also the gripper."""
    user, pw = load_creds_fr3_desk()
    rcs.control.fr3_desk.info(ip, user, pw, include_gripper)


@fr3_app.command()
def lock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Locks the robot."""
    user, pw = load_creds_fr3_desk()
    rcs.control.fr3_desk.lock(ip, user, pw)


@fr3_app.command()
def unlock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Prepares the robot by unlocking the joints and putting the robot into the FCI mode."""
    user, pw = load_creds_fr3_desk()
    rcs.control.fr3_desk.unlock(ip, user, pw)
    with rcs.control.fr3_desk.Desk(ip, user, pw) as d:
        d.activate_fci()


@fr3_app.command()
def fci(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
    shutdown: Annotated[bool, typer.Option("-s", help="After ctrl+c shuts the robot down")] = False,
):
    """Puts the robot into FCI mode, optionally unlocks the robot. Waits for ctrl+c to exit."""
    user, pw = load_creds_fr3_desk()
    try:
        with rcs.control.fr3_desk.FCI(rcs.control.fr3_desk.Desk(ip, user, pw), unlock=unlock, lock_when_done=False):
            while True:
                sleep(1)
    except KeyboardInterrupt:
        if shutdown:
            rcs.control.fr3_desk.shutdown(ip, user, pw)


@fr3_app.command()
def guiding_mode(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    disable: Annotated[bool, typer.Option("-d", help="Disable guiding mode")] = False,
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
):
    """Enables or disables guiding mode."""
    user, pw = load_creds_fr3_desk()
    rcs.control.fr3_desk.guiding_mode(ip, user, pw, disable, unlock)


@fr3_app.command()
def shutdown(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Shuts the robot down"""
    user, pw = load_creds_fr3_desk()
    rcs.control.fr3_desk.shutdown(ip, user, pw)


def main():
    main_app()
