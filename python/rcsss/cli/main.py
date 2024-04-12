from typing import Annotated, Dict, List, Optional

import rcsss
import typer
from rcsss.config import create_sample_config_yaml, read_config_yaml
from rcsss.desk import Desk
from rcsss.record import PoseList

cli = typer.Typer()


@cli.command()
def sample_config(
    path: Annotated[
        str, typer.Option("-p", help="Path to where the default config file should be saved")
    ] = "config.yaml",
):
    """Creates a sample yaml config file"""
    create_sample_config_yaml(path)


@cli.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    shut: Annotated[bool, typer.Option("-s", help="Should the robot be shut down")] = False,
):
    """Moves the FR3 to home position"""
    cfg = read_config_yaml(path)
    d = Desk(ip, cfg.hw.username, cfg.hw.password)
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
    cfg = read_config_yaml(path)
    rcsss.desk.lock(ip, cfg.hw.username, cfg.hw.password)


@cli.command()
def unlock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Prepares the robot by unlocking the joints and putting the robot into the FCI mode."""
    cfg = read_config_yaml(path)
    rcsss.desk.unlock(ip, cfg.hw.username, cfg.hw.password)


@cli.command()
def guiding_mode(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    disable: Annotated[bool, typer.Option("-d", help="Disable guiding mode")] = False,
):
    """Enables or disables guiding mode."""
    cfg = read_config_yaml(path)
    rcsss.desk.guiding_mode(ip, cfg.hw.username, cfg.hw.password, disable)


@cli.command()
def shutdown(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Shuts the robot down"""
    cfg = read_config_yaml(path)
    rcsss.desk.shutdown(ip, cfg.hw.username, cfg.hw.password)


@cli.command()
def record(
    ip_str: Annotated[str, typer.Argument(help="Name to IP dict. e.g. \"{'robot1': '192.168.100.1'}\"")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    lpaths: Annotated[Optional[List[str]], typer.Option("--lpaths", help="Paths to load n recordings")] = None,
    spath: Annotated[Optional[str], typer.Option("--spath", help="Paths to load n recordings")] = None,
):
    """Shuts the robot down"""
    cfg = read_config_yaml(path)

    ip: Dict[str, str] = eval(ip_str)

    # TODO: do not use desk functions but create an own desk instance

    if lpaths is not None:
        for r_ip in ip.values():
            rcsss.desk.unlock(r_ip, username=cfg.hw.username, password=cfg.hw.password)
        p = PoseList.load(ip, lpaths, cfg.hw.urdf_model_path)
        input("Press any key to replay")
        p.replay()
    else:
        for r_ip in ip.values():
            rcsss.desk.unlock(r_ip, username=cfg.hw.username, password=cfg.hw.password)
            rcsss.desk.guiding_mode(r_ip, username=cfg.hw.username, password=cfg.hw.password, disable=False)
        p = PoseList(ip, urdf_path=cfg.hw.urdf_model_path)
        p.record()
        if spath is not None:
            p.save(spath)


def main():
    cli()
