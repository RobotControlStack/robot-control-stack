import logging
from time import sleep
from typing import Annotated

import rcs_fr3
import typer
from rcs_fr3.desk import load_creds_fr3_desk

logger = logging.getLogger(__name__)


# FR3 CLI
fr3_app = typer.Typer(
    name="rcs.fr3",
    help=(
        "Commands to control a Franka Research 3 in RCS. "
        "This includes tools that you would usually do with Franka's Desk interface."
    ),
)


@fr3_app.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    shut: Annotated[bool, typer.Option("-s", help="Should the robot be shut down")] = False,
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
):
    """Moves the FR3 to home position"""
    user, pw = load_creds_fr3_desk()
    rcs_fr3.desk.home(ip, user, pw, shut, unlock)


@fr3_app.command()
def info(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    include_gripper: Annotated[bool, typer.Option("-g", help="includes gripper")] = False,
):
    """Prints info about the robots current joint position and end effector pose, optionally also the gripper."""
    user, pw = load_creds_fr3_desk()
    rcs_fr3.desk.info(ip, user, pw, include_gripper)


@fr3_app.command()
def lock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Locks the robot."""
    user, pw = load_creds_fr3_desk()
    rcs_fr3.desk.lock(ip, user, pw)


@fr3_app.command()
def unlock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Prepares the robot by unlocking the joints and putting the robot into the FCI mode."""
    user, pw = load_creds_fr3_desk()
    rcs_fr3.desk.unlock(ip, user, pw)
    with rcs_fr3.desk.Desk(ip, user, pw) as d:
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
        with rcs_fr3.desk.FCI(rcs_fr3.desk.Desk(ip, user, pw), unlock=unlock, lock_when_done=False):
            while True:
                sleep(1)
    except KeyboardInterrupt:
        if shutdown:
            rcs_fr3.desk.shutdown(ip, user, pw)


@fr3_app.command()
def guiding_mode(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    disable: Annotated[bool, typer.Option("-d", help="Disable guiding mode")] = False,
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
):
    """Enables or disables guiding mode."""
    user, pw = load_creds_fr3_desk()
    rcs_fr3.desk.guiding_mode(ip, user, pw, disable, unlock)


@fr3_app.command()
def shutdown(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Shuts the robot down"""
    user, pw = load_creds_fr3_desk()
    rcs_fr3.desk.shutdown(ip, user, pw)


if __name__ == "__main__":
    fr3_app()
