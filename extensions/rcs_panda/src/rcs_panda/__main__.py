import logging
from time import sleep
from typing import Annotated

import rcs_panda
import typer
from rcs_panda.desk import load_creds_franka_desk

logger = logging.getLogger(__name__)


# panda CLI
panda_app = typer.Typer(
    help=(
        "Commands to control a Franka Panda in RCS. "
        "This includes tools that you would usually do with Franka's Desk interface."
    ),
)


@panda_app.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Moves the panda to home position"""
    rcs_panda.desk.home(ip)


@panda_app.command()
def gripper(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    close_gripper: Annotated[bool, typer.Option("-c", help="close gripper")] = False,
):
    """Opens or closes the gripper"""
    rcs_panda.desk.gripper(ip, close_gripper)


@panda_app.command()
def info(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    include_gripper: Annotated[bool, typer.Option("-g", help="includes gripper")] = False,
):
    """Prints info about the robots current joint position and end effector pose, optionally also the gripper."""
    rcs_panda.desk.info(ip, include_gripper)


@panda_app.command()
def lock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Locks the robot."""
    user, pw = load_creds_franka_desk()
    rcs_panda.desk.lock(ip, user, pw)


@panda_app.command()
def unlock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Prepares the robot by unlocking the joints and putting the robot into the FCI mode."""
    user, pw = load_creds_franka_desk()
    rcs_panda.desk.unlock(ip, user, pw)
    with rcs_panda.desk.Desk(ip, user, pw) as d:
        d.activate_fci()


@panda_app.command()
def fci(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
    shutdown: Annotated[bool, typer.Option("-s", help="After ctrl+c shuts the robot down")] = False,
):
    """Puts the robot into FCI mode, optionally unlocks the robot. Waits for ctrl+c to exit."""
    user, pw = load_creds_franka_desk()
    try:
        with rcs_panda.desk.FCI(rcs_panda.desk.Desk(ip, user, pw), unlock=unlock, lock_when_done=False):
            while True:
                sleep(1)
    except KeyboardInterrupt:
        if shutdown:
            rcs_panda.desk.shutdown(ip, user, pw)


@panda_app.command()
def guiding_mode(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    disable: Annotated[bool, typer.Option("-d", help="Disable guiding mode")] = False,
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
):
    """Enables or disables guiding mode."""
    user, pw = load_creds_franka_desk()
    rcs_panda.desk.guiding_mode(ip, user, pw, disable, unlock)


@panda_app.command()
def shutdown(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Shuts the robot down"""
    user, pw = load_creds_franka_desk()
    rcs_panda.desk.shutdown(ip, user, pw)


if __name__ == "__main__":
    panda_app()
