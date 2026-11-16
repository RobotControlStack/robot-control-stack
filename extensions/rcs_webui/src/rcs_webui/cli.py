"""Command line entry point: ``rcs-webui serve``, ``rcs-webui view``, ``rcs-webui envs``, ``rcs-webui dump-config``."""

import json
import logging
import os
import signal
import threading
from pathlib import Path
from typing import Annotated, Any, Optional

import typer
from rcs_webui import configs, envs, snapshot
from rcs_webui.server import App, make_server, serve_in_thread
from rcs_webui.session import EnvSpec, SessionRunner, SessionSettings

logger = logging.getLogger("rcs_webui")

app = typer.Typer(help="RCS web interface for data collection and evaluation.", no_args_is_help=True)


def parse_overrides(items: list[str] | None) -> dict[str, Any]:
    """``key.path=value`` pairs, the value is parsed as JSON and taken as string if that fails."""
    overrides: dict[str, Any] = {}
    for item in items or []:
        key, sep, value = item.partition("=")
        if not sep or not key:
            msg = f"--set expects key.path=value, got {item!r}"
            raise typer.BadParameter(msg)
        try:
            overrides[key] = json.loads(value)
        except json.JSONDecodeError:
            overrides[key] = value
    return overrides


def run_interface(settings: SessionSettings, host: str, port: int, spec: EnvSpec | None = None):
    """Serves the web page. No env is started (unless ``spec`` is given), configs are picked in the page."""
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    settings.data_root.mkdir(parents=True, exist_ok=True)
    runner = SessionRunner(settings, spec)
    server = make_server(App(runner, settings.data_root), host, port)
    stop = threading.Event()

    def _on_signal(*_):
        # first Ctrl+C: graceful shutdown (flushes the recording); second: hard exit, so a hanging
        # close can never make the process unkillable from the terminal
        if stop.is_set():
            logger.warning("second interrupt, exiting without waiting")
            os._exit(130)
        stop.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    runner.start()
    serve_in_thread(server)
    logger.info("interface on http://%s:%d , data in %s", host, port, settings.data_root)
    stop.wait()
    logger.info("shutting down, flushing the recording")
    server.shutdown()
    runner.stop()  # closes the env and the Quest operator
    runner.wait_for_consolidation()  # a second Ctrl+C exits right away (the original files are kept)


DataRootOpt = Annotated[
    Path, typer.Option("--data-root", help="every env start creates a new folder in here, settings are kept here")
]
HostOpt = Annotated[str, typer.Option(help="address of the web server")]
PortOpt = Annotated[int, typer.Option(help="port of the web server")]
SetOpt = Annotated[
    Optional[list[str]], typer.Option("--set", help="override a config value: key.path=json_value, repeatable")
]


@app.command()
def serve(data_root: DataRootOpt = Path("rcs_data"), host: HostOpt = "0.0.0.0", port: PortOpt = 8080):
    """Web interface for data collection (Quest teleop) and policy evaluation.

    Everything else is set in the web page and kept in <data-root>/.rcs_webui/settings.json: mode, instruction,
    Quest address, config directories, policy server. No env runs until "start env" is clicked there.
    """
    run_interface(SessionSettings.load(data_root), host, port)


@app.command()
def view(data_root: DataRootOpt = Path("rcs_data"), host: HostOpt = "0.0.0.0", port: PortOpt = 8080):
    """Only the data viewer (plots, camera playback, relabel, delete), no env is started."""
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    server = make_server(App(None, data_root), host, port)
    logger.info("viewer on http://%s:%d for %s", host, port, data_root)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()


@app.command("envs")
def list_envs():
    """Lists the envs that can be started (incl. the default env packages, e.g. duobench, if installed)."""
    envs.import_env_packages(envs.DEFAULT_ENV_PACKAGES)
    for name in envs.available_envs():
        typer.echo(name)


@app.command("dump-config")
def dump_config(
    env: Annotated[str, typer.Argument(help="env name, see `rcs-webui envs`")],
    set_: SetOpt = None,
    frequency: float = 15.0,
    mode: Annotated[str, typer.Option(help="collect or eval, adapts the action space")] = "collect",
):
    """Prints the config the interface would start the env with, e.g. to keep it in a config directory."""
    envs.import_env_packages(envs.DEFAULT_ENV_PACKAGES)
    wrapped = configs.default_config(env, frequency=frequency, collect=mode == "collect", headless=True)
    wrapped["config"] = snapshot.apply_overrides(wrapped["config"], parse_overrides(set_))
    typer.echo(json.dumps(wrapped, indent=2))


def main():
    app()


if __name__ == "__main__":
    main()
