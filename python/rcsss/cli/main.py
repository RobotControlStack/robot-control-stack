from typing import Annotated, Dict, List, Optional

import rcsss
import typer
import yaml
from rcsss.record import PoseList

cli = typer.Typer()


def read_config_yaml(path: str) -> tuple[str, str]:
    with open(path, "r") as stream:
        config_dict = yaml.safe_load(stream)
    return str(config_dict["username"]), str(config_dict["password"])


@cli.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    shut: Annotated[bool, typer.Option("-s", help="Should the robot be shut down")] = False,
):
    """Moves the FR3 to home position"""
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


@cli.command()
def lock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Locks the robot."""
    username, password = read_config_yaml(path)
    rcsss.desk.lock(ip, username, password)


@cli.command()
def prepare(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    guiding_mode: Annotated[bool, typer.Option("-g", help="Should the robot be put into guiding mode")] = False,
):
    """Prepares the robot by unlocking the joints and optionally putting the robot into guiding mode."""
    username, password = read_config_yaml(path)
    rcsss.desk.prepare(ip, username, password, guiding_mode)


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

    username, password = read_config_yaml("")
    if lpaths is not None:
        for r_ip in ip.values():
            rcsss.desk.prepare(r_ip, guiding_mode=False, username=username, password=password)
        p = PoseList.load(ip, lpaths)
        input("Press any key to replay")
        p.replay()
    else:
        for r_ip in ip.values():
            rcsss.desk.prepare(r_ip, guiding_mode=True, username=username, password=password)
        p = PoseList(ip)
        p.record()
        if spath is not None:
            p.save(spath)


def main():
    cli()
