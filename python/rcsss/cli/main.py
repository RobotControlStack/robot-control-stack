from typing import Annotated, Dict, List, Optional

import rcsss
import typer
import yaml
from rcsss.desk import Desk
from rcsss.record import PoseList

cli = typer.Typer()


def read_config_yaml(path: str) -> tuple[str, str]:
    with open(path, "r") as stream:
        config_dict = yaml.safe_load(stream)
    return str(config_dict["username"]), str(config_dict["password"])


@cli.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    shut: Annotated[bool, typer.Option("-s", help="Should the robot be shut down")] = False,
):
    """Moves the FR3 to home position"""
    username, password = read_config_yaml(path)
    d = Desk(ip, username, password)
    d.take_control(force=True)
    d.disable_guiding_mode()
    d.activate_fci()

    f = rcsss.hw.FR3(ip)
    config = rcsss.hw.FR3Config()
    config.speed_factor = 0.7
    config.controller = rcsss.hw.IKController.internal
    config.guiding_mode_enabled = False
    f.set_parameters(config)
    g = rcsss.hw.FrankaHand(ip)
    if shut:
        g.shut()
    else:
        g.release()
    f.move_home()
    d.release_control()


@cli.command()
def lock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Locks the robot."""
    username, password = read_config_yaml(path)
    rcsss.desk.lock(ip, username, password)


@cli.command()
def unlock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Prepares the robot by unlocking the joints and putting the robot into the FCI mode."""
    username, password = read_config_yaml(path)
    rcsss.desk.unlock(ip, username, password)


@cli.command()
def gm(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    disable: Annotated[bool, typer.Option("-d", help="Disable guiding mode")] = False,
):
    """Enables or disables guiding mode."""
    username, password = read_config_yaml(path)
    rcsss.desk.guiding_mode(ip, username, password, disable)


@cli.command()
def shutdown(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Shuts the robot down"""
    username, password = read_config_yaml(path)
    rcsss.desk.shutdown(ip, username, password)


@cli.command()
def record(
    ip_str: Annotated[str, typer.Argument(help="Name to IP dict. e.g. \"{'robot1': '192.168.100.1'}\"")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    lpaths: Annotated[Optional[List[str]], typer.Option("--lpaths", help="Paths to load n recordings")] = None,
    spath: Annotated[Optional[str], typer.Option("--spath", help="Paths to load n recordings")] = None,
):
    """Shuts the robot down"""
    username, password = read_config_yaml(path)

    ip: Dict[str, str] = eval(ip_str)

    # TODO: do not use desk functions but create an own desk instance

    if lpaths is not None:
        for r_ip in ip.values():
            rcsss.desk.unlock(r_ip, username=username, password=password)
        p = PoseList.load(ip, lpaths)
        input("Press any key to replay")
        p.replay()
    else:
        for r_ip in ip.values():
            rcsss.desk.unlock(r_ip, username=username, password=password)
            rcsss.desk.guiding_mode(r_ip, username=username, password=password, disable=False)
        p = PoseList(ip)
        p.record()
        if spath is not None:
            p.save(spath)


def main():
    cli()
