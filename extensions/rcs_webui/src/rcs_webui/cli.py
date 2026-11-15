"""Command line entry point: ``rcs-webui collect ...``, ``rcs-webui envs``, ``rcs-webui dump-config ...``."""

import json
import logging
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


def _make_operator(settings: SessionSettings, is_sim: bool):
    if settings.operator == "none":
        return None
    if settings.operator != "quest":
        msg = f"unknown operator {settings.operator!r}, use quest or none"
        raise typer.BadParameter(msg)
    from rcs.operator.quest import QuestConfig, QuestOperator

    config = QuestConfig(
        mq3_addr=settings.quest_addr,
        simulation=is_sim,
        switched_left_right=settings.quest_switched_left_right,
        display_cameras=False,
        read_frequency=90,
    )
    operator = QuestOperator(config)
    operator.start()
    return operator


def run_interface(settings: SessionSettings, spec: EnvSpec, host: str, port: int):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    settings.data_root.mkdir(parents=True, exist_ok=True)
    is_sim = spec.env is not None and spec.env.startswith("rcs/")
    if spec.from_recording is not None:
        _, meta = snapshot.load_session_config(spec.from_recording)
        is_sim = str(meta.get("creator", "")).startswith("rcs.envs.")
    elif spec.config_file is not None:
        is_sim = configs.read_file(spec.config_file, spec.env)[0].startswith("rcs.envs.")
    operator = _make_operator(settings, is_sim)
    runner = SessionRunner(settings, spec, operator)
    server = make_server(App(runner, settings.data_root), host, port)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    runner.start()
    serve_in_thread(server)
    logger.info("interface on http://%s:%d , data in %s", host, port, settings.data_root)
    stop.wait()
    logger.info("shutting down, flushing the recording")
    server.shutdown()
    runner.stop()
    if operator is not None:
        operator.close()


EnvOpt = Annotated[
    Optional[str], typer.Option("--env", "-e", help="env to start, see `rcs-webui envs` (e.g. rcs/fr3 or fr3)")
]
ConfigOpt = Annotated[
    Optional[Path],
    typer.Option("--config", help="env config JSON from `rcs-webui dump-config` (a bare config also needs --env)"),
]
FromRecOpt = Annotated[
    Optional[Path], typer.Option("--from-recording", help="session folder whose env_config.json is used")
]
SetOpt = Annotated[
    Optional[list[str]], typer.Option("--set", help="override a config value: key.path=json_value, repeatable")
]
DataRootOpt = Annotated[Path, typer.Option("--data-root", help="every env start creates a new folder in here")]
ConfigDirOpt = Annotated[
    Optional[list[Path]],
    typer.Option("--config-dir", help="directory with config files (*.json) offered in the web page, repeatable"),
]


@app.command()
def collect(
    env: EnvOpt = None,
    config: ConfigOpt = None,
    from_recording: FromRecOpt = None,
    set_: SetOpt = None,
    data_root: DataRootOpt = Path("rcs_data"),
    config_dir: ConfigDirOpt = None,
    instruction: Annotated[str, typer.Option(help="language instruction stored with every step")] = "",
    frequency: Annotated[float, typer.Option(help="env step rate in Hz (default configs only)")] = 15.0,
    operator: Annotated[str, typer.Option(help="quest or none (web buttons only)")] = "quest",
    quest_addr: Annotated[str, typer.Option(help="IP of this PC in the network of the Quest")] = "10.42.0.1",
    quest_switch_left_right: bool = False,
    gui: Annotated[bool, typer.Option(help="open the MuJoCo viewer (sim only)")] = False,
    host: Annotated[str, typer.Option(help="address of the web server")] = "0.0.0.0",
    port: Annotated[int, typer.Option(help="port of the web server")] = 8080,
):
    """Teleoperated data collection: Quest A starts an episode, B ends it successful, Y unsuccessful."""
    if sum(x is not None for x in (config, from_recording)) > 1:
        msg = "use either --config or --from-recording"
        raise typer.BadParameter(msg)
    if env is None and from_recording is None and config is None:
        msg = "--env, --config or --from-recording is required"
        raise typer.BadParameter(msg)
    settings = SessionSettings(
        data_root=data_root,
        mode="collect",
        instruction=instruction,
        frequency=frequency,
        headless=not gui,
        operator=operator,
        quest_addr=quest_addr,
        quest_switched_left_right=quest_switch_left_right,
        config_dirs=list(config_dir or []),
    )
    spec = EnvSpec(env=env, config_file=config, from_recording=from_recording, overrides=parse_overrides(set_))
    run_interface(settings, spec, host, port)


@app.command()
def view(
    data_root: DataRootOpt = Path("rcs_data"),
    host: Annotated[str, typer.Option(help="address of the web server")] = "0.0.0.0",
    port: Annotated[int, typer.Option(help="port of the web server")] = 8080,
):
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
    """Lists the envs that can be started."""
    for name in envs.available_envs():
        typer.echo(name)


@app.command("dump-config")
def dump_config(
    env: Annotated[str, typer.Argument(help="env name, see `rcs-webui envs`")],
    set_: SetOpt = None,
    frequency: float = 15.0,
    mode: Annotated[str, typer.Option(help="collect or eval, adapts the action space")] = "collect",
):
    """Prints the config the interface would start the env with, edit it and pass it with --config."""
    wrapped = configs.default_config(env, frequency=frequency, collect=mode == "collect", headless=True)
    wrapped["config"] = snapshot.apply_overrides(wrapped["config"], parse_overrides(set_))
    typer.echo(json.dumps(wrapped, indent=2))


def main():
    app()


if __name__ == "__main__":
    main()
