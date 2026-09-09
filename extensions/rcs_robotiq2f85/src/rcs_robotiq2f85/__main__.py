import logging

import typer
from robotiq2f import LinuxFindTTYWithSerialNumber

logger = logging.getLogger(__name__)
# callback=... keeps `serials` a subcommand; typer collapses a single-command app otherwise.
robotiq2f85_app = typer.Typer(help="CLI tool for the Robotiq 2F gripper module of rcs.", callback=lambda: None)


@robotiq2f85_app.command()
def serials():
    """Reads out the serial numbers of the connected serial devices."""
    devices = LinuxFindTTYWithSerialNumber().list_devices()

    if len(devices) == 0:
        typer.secho("No serial devices connected.", fg=typer.colors.YELLOW, err=True)
        return

    typer.echo("Connected devices:")
    for port, serial in devices:
        typer.echo(f"  {port}: {serial if serial is not None else 'unknown'}")


def main():
    robotiq2f85_app()


if __name__ == "__main__":
    main()
