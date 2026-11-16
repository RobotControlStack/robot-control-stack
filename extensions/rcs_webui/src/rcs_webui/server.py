"""HTTP server of the interface: a JSON API plus a single static page, standard library only."""

import json
import logging
import mimetypes
import re
import threading
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

import numpy as np
import simplejpeg
from rcs_webui import configs, data, envs
from rcs_webui.policy import available_policy_clients, load_policy_client_class
from rcs_webui.session import EnvSpec, PolicySettings, SessionRunner

logger = logging.getLogger(__name__)

Route = Callable[["Handler", re.Match, dict[str, Any]], Any]

RUNNING_ID = "running:"


class ApiError(Exception):
    def __init__(self, status: HTTPStatus, message: str):
        super().__init__(message)
        self.status = status


class App:
    """Routes of the interface. ``runner`` is None if the interface only shows recorded data."""

    def __init__(self, runner: SessionRunner | None, data_root: Path):
        self.runner = runner
        self.data_root = Path(data_root)
        self.get_routes: list[tuple[re.Pattern, Route]] = []
        self.post_routes: list[tuple[re.Pattern, Route]] = []
        self._register()

    def get(self, pattern: str):
        def deco(fn: Route) -> Route:
            self.get_routes.append((re.compile(f"^{pattern}$"), fn))
            return fn

        return deco

    def post(self, pattern: str):
        def deco(fn: Route) -> Route:
            self.post_routes.append((re.compile(f"^{pattern}$"), fn))
            return fn

        return deco

    def _need_runner(self) -> SessionRunner:
        if self.runner is None:
            raise ApiError(HTTPStatus.CONFLICT, "no env session is running (viewer only mode)")
        return self.runner

    def _session(self, match: re.Match) -> Path:
        try:
            return data.session_path(self.data_root, unquote(match["session"]))
        except data.DataError as e:
            raise ApiError(HTTPStatus.NOT_FOUND, str(e)) from e

    def _config_entry(self, entry_id: str) -> dict[str, Any]:
        runner = self._need_runner()
        if entry_id == RUNNING_ID:
            if runner.config_json is None:
                raise ApiError(HTTPStatus.NOT_FOUND, "no env is running")
            return configs.normalized(runner.config_json["creator"], runner.config_json["config"])
        envs.import_env_packages(runner.settings.env_packages)
        try:
            return configs.load_entry(
                entry_id,
                config_dirs=runner.settings.config_dirs,
                data_root=self.data_root,
                frequency=runner.settings.frequency,
                collect=runner.settings.mode == "collect",
                headless=runner.settings.headless,
            )
        except Exception as e:
            raise ApiError(HTTPStatus.BAD_REQUEST, f"{type(e).__name__}: {e}") from e

    def _check_not_recording(self, uuid: str):
        if self.runner is not None and self.runner.status().get("episode_uuid") == uuid:
            raise ApiError(HTTPStatus.CONFLICT, "the episode is being recorded, end it first")

    def _register(self):
        @self.get("/api/status")
        def status(_h, _m, _q):
            if self.runner is None:
                return {"state": "viewer", "data_root": str(self.data_root)}
            return {**self.runner.status(), "data_root": str(self.data_root), "cameras": self.runner.cameras()}

        @self.get("/api/envs")
        def list_envs(_h, _m, _q):
            return {"envs": envs.available_envs()}

        @self.get("/api/sessions")
        def sessions(_h, _m, _q):
            return {"sessions": data.list_sessions(self.data_root)}

        @self.get(r"/api/sessions/(?P<session>[^/]+)/episodes")
        def episodes(_h, m, _q):
            return {"episodes": data.list_episodes(self._session(m))}

        @self.get(r"/api/sessions/(?P<session>[^/]+)/episodes/(?P<uuid>[0-9a-f]+)")
        def episode(_h, m, _q):
            return data.episode_signals(self._session(m), m["uuid"])

        @self.get(
            r"/api/sessions/(?P<session>[^/]+)/episodes/(?P<uuid>[0-9a-f]+)/frames/(?P<camera>[^/]+)/(?P<step>\d+)\.jpg"
        )
        def episode_frame(h, m, _q):
            jpeg = data.frame(self._session(m), m["uuid"], unquote(m["camera"]), int(m["step"]))
            h.send_bytes(jpeg, "image/jpeg")

        @self.post(r"/api/sessions/(?P<session>[^/]+)/episodes/(?P<uuid>[0-9a-f]+)/label")
        def label(_h, m, body):
            self._check_not_recording(m["uuid"])
            if not isinstance(body.get("success"), bool):
                raise ApiError(HTTPStatus.BAD_REQUEST, "success must be true or false")
            return {"files": data.relabel_episode(self._session(m), m["uuid"], body["success"])}

        @self.post(r"/api/sessions/(?P<session>[^/]+)/episodes/(?P<uuid>[0-9a-f]+)/delete")
        def delete(_h, m, _body):
            self._check_not_recording(m["uuid"])
            return {"files": data.delete_episode(self._session(m), m["uuid"])}

        @self.get(r"/api/live/(?P<camera>[^/]+)\.jpg")
        def live_frame(h, m, _q):
            frame = self._need_runner().latest_frame(unquote(m["camera"]))
            if frame is None:
                raise ApiError(HTTPStatus.NOT_FOUND, "no frame")
            h.send_bytes(simplejpeg.encode_jpeg(np.ascontiguousarray(frame), quality=70), "image/jpeg")

        @self.post("/api/button")
        def button(_h, _m, body):
            name = str(body.get("button", "")).upper()
            if name not in ("A", "B", "Y", "X"):
                raise ApiError(HTTPStatus.BAD_REQUEST, "button must be A, B, Y or X")
            self._need_runner().submit("button", name)
            return {"ok": True}

        @self.get("/api/policy/clients")
        def policy_clients(_h, _m, _q):
            return {"clients": available_policy_clients()}

        @self.post("/api/policy")
        def policy(_h, _m, body):
            runner = self._need_runner()
            if runner.settings.mode != "eval":
                raise ApiError(HTTPStatus.CONFLICT, "the policy server is only used in eval mode")
            current = runner.settings.policy or PolicySettings()
            try:
                execute_steps = body.get("execute_steps", current.execute_steps)
                settings = PolicySettings(
                    client=str(body.get("client", current.client)),
                    host=str(body.get("host", current.host)),
                    port=int(body.get("port", current.port)),
                    options=dict(body.get("options", current.options)),
                    execute_steps=int(execute_steps) if execute_steps not in (None, "") else None,
                    rtc_delay=int(body.get("rtc_delay", current.rtc_delay)),
                )
                cls = load_policy_client_class(settings.client)
            except (TypeError, ValueError, ImportError) as e:
                raise ApiError(HTTPStatus.BAD_REQUEST, str(e)) from e
            if settings.rtc_delay > 0 and not cls.capabilities.realtime_chunking:
                raise ApiError(HTTPStatus.BAD_REQUEST, f"{settings.client} does not support real-time chunking")
            runner.submit("policy", settings)
            return {"ok": True}

        @self.post("/api/instruction")
        def instruction(_h, _m, body):
            self._need_runner().submit("instruction", body.get("instruction", ""))
            return {"ok": True}

        @self.get("/api/settings")
        def get_settings(_h, _m, _q):
            runner = self._need_runner()
            return {
                "settings": runner.settings.to_json(),
                "env_start_keys": list(runner.settings.ENV_START_KEYS),
                "env_running": runner.env_running,
                "env_packages": envs.import_env_packages(runner.settings.env_packages),
                "settings_file": str(runner.settings.settings_file),
            }

        @self.post("/api/settings")
        def set_settings(_h, _m, body):
            try:
                self._need_runner().update_settings(body)
            except (ValueError, TypeError) as e:
                raise ApiError(
                    HTTPStatus.CONFLICT if "stop the env" in str(e) else HTTPStatus.BAD_REQUEST, str(e)
                ) from e
            return {"ok": True}

        @self.get("/api/configs")
        def config_catalog(_h, _m, _q):
            if self.runner is None:
                return {"configs": configs.catalog([], self.data_root)}
            envs.import_env_packages(self.runner.settings.env_packages)
            entries = configs.catalog(self.runner.settings.config_dirs, self.data_root)
            if self.runner.config_json is not None:
                label = Path(self.runner.status().get("session_dir") or "running").name
                entries.insert(0, {"id": RUNNING_ID, "group": "running env", "label": f"running: {label}"})
            return {"configs": entries, "last": self.runner.settings.last_config}

        @self.get("/api/configs/entry")
        def config_entry(_h, _m, query):
            return self._config_entry(query.get("id", ""))

        @self.post("/api/configs/validate")
        def config_validate(_h, _m, body):
            if self.runner is not None:
                envs.import_env_packages(self.runner.settings.env_packages)
            try:
                cfg = configs.validate(body)
                creator = configs.unwrap(body)[0]
            except Exception as e:
                return {"ok": False, "error": f"{type(e).__name__}: {e}"}
            return {"ok": True, "type": type(cfg).__name__, "summary": envs.describe(cfg, creator)}

        @self.post("/api/stop_env")
        def stop_env(_h, _m, _body):
            self._need_runner().submit("stop_env")
            return {"ok": True}

        @self.post("/api/restart")
        def restart(_h, _m, body):
            runner = self._need_runner()
            if "config" in body:
                envs.import_env_packages(runner.settings.env_packages)
                try:
                    configs.validate(body["config"])
                except Exception as e:
                    raise ApiError(HTTPStatus.BAD_REQUEST, f"invalid config: {type(e).__name__}: {e}") from e
                source: dict[str, Any] = {}
                base_id = body.get("base_id")
                if base_id:
                    source["base"] = base_id
                    try:
                        base = self._config_entry(base_id)
                        source["changed"] = configs.changed_paths(base["config"], body["config"]["config"])
                    except ApiError:
                        logger.warning("base config %s not loadable, no diff stored", base_id)
                    if base_id != RUNNING_ID:
                        runner.update_settings({"last_config": base_id})
                runner.submit("restart", EnvSpec(config_data=body["config"], source=source))
                return {"ok": True}
            overrides = body.get("overrides") or {}
            if not isinstance(overrides, dict):
                raise ApiError(HTTPStatus.BAD_REQUEST, "overrides must be an object of dotted paths to values")
            from_recording = body.get("from_recording")
            if from_recording:
                path = (self.data_root / from_recording).resolve()
                if self.data_root.resolve() not in path.parents:
                    raise ApiError(HTTPStatus.BAD_REQUEST, "from_recording must be a session in the data root")
                spec = EnvSpec(from_recording=path, overrides=overrides)
            elif body.get("env"):
                spec = EnvSpec(env=body["env"], overrides=overrides)
            else:
                spec = EnvSpec(
                    env=runner.env_spec.env,
                    config_file=runner.env_spec.config_file,
                    from_recording=runner.env_spec.from_recording,
                    overrides={**runner.env_spec.overrides, **overrides},
                )
            runner.submit("restart", spec)
            return {"ok": True}


class Handler(BaseHTTPRequestHandler):
    app: App
    server_version = "rcs-webui"

    def log_message(self, format: str, *args: Any):
        logger.debug("%s %s", self.address_string(), format % args)

    def send_bytes(self, data: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK):
        try:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            # the browser gave up on the request (e.g. a camera image replaced while the page re-renders after an
            # env stop): nothing to answer, not an error
            logger.debug("client closed the connection before %s was answered", self.path)
            self.close_connection = True

    def send_json(self, value: Any, status: HTTPStatus = HTTPStatus.OK):
        self.send_bytes(json.dumps(value, default=_json_default).encode(), "application/json", status)

    def _dispatch(self, routes: list[tuple[re.Pattern, Route]], body: dict[str, Any]):
        path = urlparse(self.path).path
        for pattern, fn in routes:
            match = pattern.match(path)
            if match:
                try:
                    result = fn(self, match, body)
                except ApiError as e:
                    self.send_json({"error": str(e)}, e.status)
                    return
                except data.DataError as e:
                    self.send_json({"error": str(e)}, HTTPStatus.NOT_FOUND)
                    return
                except Exception as e:
                    logger.exception("request %s failed", path)
                    self.send_json({"error": f"{type(e).__name__}: {e}"}, HTTPStatus.INTERNAL_SERVER_ERROR)
                    return
                if result is not None:
                    self.send_json(result)
                return
        if self.command == "GET" and self._static(path):
            return
        self.send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _static(self, path: str) -> bool:
        name = "index.html" if path in ("", "/") else path.lstrip("/")
        if "/" in name or name.startswith("."):
            return False
        static = resources.files("rcs_webui") / "static" / name
        if not static.is_file():
            return False
        content_type = mimetypes.guess_type(name)[0] or "application/octet-stream"
        self.send_bytes(static.read_bytes(), content_type)
        return True

    def do_GET(self):
        query = {k: v[-1] for k, v in parse_qs(urlparse(self.path).query).items()}
        self._dispatch(self.app.get_routes, query)

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw or b"{}")
        except json.JSONDecodeError:
            self.send_json({"error": "body must be JSON"}, HTTPStatus.BAD_REQUEST)
            return
        self._dispatch(self.app.post_routes, body if isinstance(body, dict) else {})


def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return str(value)


def make_server(app: App, host: str, port: int) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (Handler,), {"app": app})
    server = ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    return server


def serve_in_thread(server: ThreadingHTTPServer) -> threading.Thread:
    thread = threading.Thread(target=server.serve_forever, daemon=True, name="rcs-webui-http")
    thread.start()
    return thread
