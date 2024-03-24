from typing import Annotated
import typer
import rcsss
import yaml

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
    f = rcsss.common.FR3(ip)
    config = rcsss.hw.FR3Config()
    config.speed_scaling = 0.7
    config.controller = rcsss.hw.IKController.internal
    config.guiding_mode_enabled = False
    f.set_parameters(config)
    g = rcsss.common.FrankaHand(ip)
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


def main():
    cli()
