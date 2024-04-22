from contextlib import ExitStack
from typing import Annotated, Optional

import rcsss
import typer
from rcsss.config import create_sample_config_yaml, read_config_yaml
from rcsss.record import PoseList

main_app = typer.Typer(help="CLI tool for the Robot Control Stack (RCS).")

fr3_app = typer.Typer()
main_app.add_typer(
    fr3_app,
    name="fr3",
    help="Commands to control a Franka Research 3. This includes tools that you would usually do with Franka's Desk interface.",
)


@main_app.command()
def sample_config(
    path: Annotated[
        str, typer.Option("-p", help="Path to where the default config file should be saved")
    ] = "config.yaml",
):
    """Creates a sample yaml config file"""
    create_sample_config_yaml(path)


@fr3_app.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    shut: Annotated[bool, typer.Option("-s", help="Should the robot be shut down")] = False,
):
    """Moves the FR3 to home position"""
    cfg = read_config_yaml(path)
    rcsss.desk.home(ip, cfg.hw.username, cfg.hw.password, shut)


@fr3_app.command()
def lock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Locks the robot."""
    cfg = read_config_yaml(path)
    rcsss.desk.lock(ip, cfg.hw.username, cfg.hw.password)


@fr3_app.command()
def unlock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Prepares the robot by unlocking the joints and putting the robot into the FCI mode."""
    cfg = read_config_yaml(path)
    rcsss.desk.unlock(ip, cfg.hw.username, cfg.hw.password)


@fr3_app.command()
def guiding_mode(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    disable: Annotated[bool, typer.Option("-d", help="Disable guiding mode")] = False,
):
    """Enables or disables guiding mode."""
    cfg = read_config_yaml(path)
    rcsss.desk.guiding_mode(ip, cfg.hw.username, cfg.hw.password, disable)


@fr3_app.command()
def shutdown(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Shuts the robot down"""
    cfg = read_config_yaml(path)
    rcsss.desk.shutdown(ip, cfg.hw.username, cfg.hw.password)


@fr3_app.command()
def record(
    ip_str: Annotated[str, typer.Argument(help="Name to IP dict. e.g. \"{'robot1': '192.168.100.1'}\"")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    lpaths: Annotated[Optional[list[str]], typer.Option("--lpaths", help="Paths to load n recordings")] = None,
    spath: Annotated[Optional[str], typer.Option("--spath", help="Paths to load n recordings")] = None,
):
    """Tool to record poses with multiple FR3 robots."""
    cfg = read_config_yaml(path)

    ip: dict[str, str] = eval(ip_str)

    if lpaths is not None:
        with ExitStack() as stack:
            for r_ip in ip.values():
                stack.enter_context(
                    rcsss.desk.Desk.fci(r_ip, username=cfg.hw.username, password=cfg.hw.password, unlock=True)
                )

            p = PoseList.load(ip, lpaths, cfg.hw.urdf_model_path)
            input("Press any key to replay")
            p.replay()
    else:
        with ExitStack() as stack:
            for r_ip in ip.values():
                stack.enter_context(
                    rcsss.desk.Desk.guiding_mode(r_ip, username=cfg.hw.username, password=cfg.hw.password, unlock=True)
                )

            p = PoseList(ip, urdf_path=cfg.hw.urdf_model_path)
            p.record()
            if spath is not None:
                p.save(spath)


def main():
    main_app()
