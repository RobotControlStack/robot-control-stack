import base64
import dataclasses
import hashlib
import json as json_module
import logging
import os
import ssl
import threading
import time
from typing import Callable, Literal
from urllib import parse

import rcsss
import requests
from rcsss.envs.factories import default_fr3_hw_gripper_cfg
from requests.packages import urllib3  # type: ignore[attr-defined]
from websockets.sync.client import connect

_logger = logging.getLogger("desk")

TOKEN_PATH = "~/.rcsss/token.conf"
"""
Path to the configuration file holding known control tokens.
If :py:class:`Desk` is used to connect to a control unit's
web interface and takes control, the generated token is stored
in this file under the unit's IP address or hostname.
"""


def home(ip: str, username: str, password: str, shut: bool, unlock: bool = False):
    with Desk.fci(ip, username, password, unlock=unlock):
        f = rcsss.hw.FR3(ip)
        config = rcsss.hw.FR3Config()
        config.speed_factor = 0.7
        config.ik_solver = rcsss.hw.IKSolver.franka
        config.guiding_mode_enabled = True
        f.set_parameters(config)
        config_hand = rcsss.hw.FHConfig()
        g = rcsss.hw.FrankaHand(ip, config_hand)
        if shut:
            g.shut()
        else:
            g.open()
        f.move_home()


def info(ip: str, username: str, password: str, include_hand: bool = False):
    with Desk.fci(ip, username, password):
        f = rcsss.hw.FR3(ip)
        config = rcsss.hw.FR3Config()
        f.set_parameters(config)
        print("Robot info:")
        print("Current cartesian position:")
        print(f.get_cartesian_position())
        print("Current joint position:")
        print(f.get_joint_position())
        if include_hand:
            config_hand = default_fr3_hw_gripper_cfg()
            g = rcsss.hw.FrankaHand(ip, config_hand)
            print("Gripper info:")
            print("Current normalized width:")
            print(g.get_normalized_width())


def lock(ip: str, username: str, password: str):
    with Desk(ip, username, password) as d:
        d.lock()


def unlock(ip: str, username: str, password: str):
    with Desk(ip, username, password) as d:
        d.unlock()


def guiding_mode(ip: str, username: str, password: str, disable: bool = False, unlock=False):
    with Desk(ip, username, password) as d:
        if disable:
            d.disable_guiding_mode()
            if unlock:
                d.lock()
        else:
            if unlock:
                d.unlock()
            d.enable_guiding_mode()


def shutdown(ip: str, username: str, password: str):
    d = Desk(ip, username, password)
    d.take_control(force=True)
    d.lock()
    d.shutdown()


@dataclasses.dataclass
class Token:
    """
    Represents a Desk token owned by a user.
    """

    id: str = ""
    owned_by: str = ""
    token: str = ""


class DummyResourceManager:
    def __enter__(self):
        pass

    def __exit__(self, *args):
        pass


class Desk:
    """
    Connects to the control unit running the web-based Desk interface
    to manage the robot. Use this class to interact with the Desk
    from Python, e.g. if you use a headless setup. This interface
    supports common tasks such as unlocking the brakes, activating
    the FCI etc.

    Newer versions of the system software use role-based access
    management to allow only one user to be in control of the Desk
    at a time. The controlling user is authenticated using a token.
    The :py:class:`Desk` class saves those token in :py:obj:`TOKEN_PATH`
    and will use them when reconnecting to the Desk, retaking control.
    Without a token, control of a Desk can only be taken, if there is
    no active claim or the controlling user explicitly relinquishes control.
    If the controlling user's token is lost, a user can take control
    forcefully (cf. :py:func:`Desk.take_control`) but needs to confirm
    physical access to the robot by pressing the circle button on the
    robot's Pilot interface.

    Can also be used as a context manager to ensure that control is
    taken and released correctly.
    """

    # TODO: method that checks if robot is on?

    def __init__(self, hostname: str, username: str, password: str) -> None:
        urllib3.disable_warnings()
        self._session = requests.Session()
        self._session.verify = False
        self._hostname = hostname
        self._username = username
        self._password = password
        self._logged_in = False
        self._token = Token()
        self._listening = False
        self._listen_thread: threading.Thread | None = None

        # the following variables might be out of sync
        # TODO: is there a way to check for the robot's state?
        self.guiding_mode_enabled = False
        self.fci_enabled = False
        self.locked = True
        self._button_states = {
            "circle": False,
            "cross": False,
            "check": False,
            "left": False,
            "right": False,
            "down": False,
            "up": False,
        }
        # Create an SSLContext that doesn't verify certificates
        self.ssl_context = ssl.create_default_context()
        self.ssl_context.check_hostname = False  # Disable hostname verification
        self.ssl_context.verify_mode = ssl.CERT_NONE  # Disable certificate verification

        self.login()

    def __enter__(self) -> "Desk":
        self.take_control(force=True)
        return self

    def __exit__(self, *args):
        self.stop_listen()
        self.release_control()

    @classmethod
    def fci(
        cls,
        hostname: str,
        username: str,
        password: str,
        unlock: bool = False,
        lock_when_done=False,
        guiding_mode_when_done=True,
    ) -> "FCI":
        return FCI(
            cls(hostname, username, password),
            unlock,
            lock_when_done=lock_when_done,
            guiding_mode_when_done=guiding_mode_when_done,
        )

    @classmethod
    def guiding_mode(cls, hostname: str, username: str, password: str, unlock: bool = False) -> "GuidingMode":
        return GuidingMode(cls(hostname, username, password), unlock)

    def lock(self, force: bool = True) -> None:
        """
        Locks the brakes. API call blocks until the brakes are locked.
        """
        self._request("post", "/desk/api/joints/lock", files={"force": force})  # type: ignore[dict-item]
        self.locked = True

    def unlock(self, force: bool = True) -> None:
        """
        Unlocks the brakes. API call blocks until the brakes are unlocked.
        """
        self._request(
            "post",
            "/desk/api/joints/unlock",
            files={"force": force},  # type: ignore[dict-item]
            headers={"X-Control-Token": self._token.token},
        )
        self.locked = False

    def enable_guiding_mode(self) -> None:
        """
        Enables guiding mode which deactivates robot control.
        """
        self._request(
            "post",
            "/desk/api/operating-mode/programming",
            headers={"X-Control-Token": self._token.token},
        )
        self.guiding_mode_enabled = True

    def disable_guiding_mode(self) -> None:
        """
        Disables guiding mode and activates robot control.
        """
        self._request(
            "post",
            "/desk/api/operating-mode/execution",
            headers={"X-Control-Token": self._token.token},
        )
        self.guiding_mode_enabled = False

    def reboot(self) -> None:
        """
        Reboots the robot hardware (this will close open connections).
        """
        self._request("post", "/admin/api/reboot", headers={"X-Control-Token": self._token.token})
        self.guiding_mode_enabled = False
        self.fci_enabled = False

    def shutdown(self) -> None:
        """
        Reboots the robot hardware (this will close open connections).
        """
        self._request(
            "post",
            "/admin/api/shutdown",
            headers={"X-Control-Token": self._token.token},
        )
        self.guiding_mode_enabled = False
        self.fci_enabled = False

    def activate_fci(self) -> None:
        """
        Activates the Franka Research Interface (FCI). Note that the
        brakes must be unlocked first. For older Desk versions, this
        function does nothing.
        """
        self._request(
            "post",
            "/desk/api/system/fci",
            headers={"X-Control-Token": self._token.token},
        )
        # sleep needed to make sure fci has really been activated on the frankas side
        time.sleep(0.5)
        self.fci_enabled = True

    def deactivate_fci(self) -> None:
        """
        Deactivates the Franka Research Interface (FCI). For older
        Desk versions, this function does nothing.
        """
        self._request(
            "delete",
            "/admin/api/control-token/fci",
            headers={"X-Control-Token": self._token.token},
            json={"token": self._token.token},
        )
        self.fci_enabled = False

    def take_control(self, force: bool = False) -> bool:
        """
        Takes control of the Desk, generating a new control token and saving it.
        If `force` is set to True, control can be taken forcefully even if another
        user is already in control. However, the user will have to press the circle
        button on the robot's Pilot within an alotted amount of time to confirm
        physical access.

        For legacy versions of the Desk, this function does nothing.
        """
        active = self._get_active_token()

        # try to read token from cache file
        token_path = os.path.expanduser('~/.cache/rcs_fr3_token')
        if active.id != "" and self._token.id == "" and os.path.exists(token_path):
            with open(token_path, "r") as f:
                content = f.read()
            content = content.split("/n")
            self._token = Token(content[0], content[1], content[2])
            


        # we already have control
        if active.id != "" and self._token.id == active.id:
            _logger.info("Retaken control.")
            return True

        # someone else has control and we dont want to force
        if active.id != "" and not force:
            _logger.warning("Cannot take control. User %s is in control.", active.owned_by)
            return False

        response = self._request(
            "post",
            f'/admin/api/control-token/request{"?force" if force else ""}',
            json={"requestedBy": self._username},
        ).json()

        if active.id == "":
            _logger.info("No active token.")
        else:
            # someone else has control and want to force
            timeout = self._request("get", "/admin/api/safety").json()["tokenForceTimeout"]
            _logger.warning(
                "You have %d seconds to confirm control by pressing circle button on robot.",
                timeout,
            )
            with connect(
                f"wss://{self._hostname}/desk/api/navigation/events",
                server_hostname="robot.franka.de",
                additional_headers={"authorization": self._session.cookies.get("authorization")},  # type: ignore[arg-type]
                ssl_context=self.ssl_context,
            ) as websocket:
                while True:
                    event: dict = json_module.loads(websocket.recv(timeout))
                    if event["circle"]:
                        break
        self._token = Token(str(response["id"]), self._username, response["token"])
        with open(token_path, "w") as f:
            f.write("/n".join([self._token.id, self._token.owned_by, self._token.token]))
        _logger.info("Taken control.")
        return True

    def release_control(self) -> None:
        """
        Explicitly relinquish cofilentrol of the Desk. This will allow
        other users to take control or transfer control to the next
        user if there is an active queue of control requests.
        """
        _logger.info("Releasing control.")
        try:
            self._request(
                "delete",
                "/admin/api/control-token",
                headers={"X-Control-Token": self._token.token},
                json={"token": self._token.token},
            )
        except ConnectionError as err:
            if "ControlTokenUnknown" in str(err):
                _logger.warning("Control release failed. Not in control.")
            else:
                raise err
        self._token = Token()

    @staticmethod
    def encode_password(username: str, password: str) -> str:
        """
        Encodes the password into the form needed to log into the Desk interface.
        """
        bytes_str = ",".join(
            [str(b) for b in hashlib.sha256((f"{password}#{username}@franka").encode("utf-8")).digest()]
        )
        return base64.encodebytes(bytes_str.encode("utf-8")).decode("utf-8")

    def login(self) -> None:
        """
        Uses the object's instance parameters to log into the Desk.
        The :py:class`Desk` class's constructor will try to connect
        and login automatically.
        """
        login = self._request(
            "post",
            "/admin/api/login",
            json={
                "login": self._username,
                "password": self.encode_password(self._username, self._password),
            },
        )
        self._session.cookies.set("authorization", login.text)
        self._logged_in = True
        _logger.info("Login successful.")

    def logout(self) -> None:
        """
        Logs the current user out of the Desk. API calls will no longer
        be possible.
        """
        self._request("post", "/admin/api/logout")
        self._session.cookies.clear()
        self._logged_in = False
        _logger.info("Logout successful.")

    def _get_active_token(self) -> Token:
        token = Token()
        response = self._request("get", "/admin/api/control-token").json()
        if response["activeToken"] is not None:
            token.id = str(response["activeToken"]["id"])
            token.owned_by = response["activeToken"]["ownedBy"]
        return token

    def has_control(self) -> bool:
        """
        Returns:
          bool: True if this instance is in control of the Desk.
        """
        return self._token.id == self._get_active_token().id

    def _request(
        self,
        method: Literal["post", "get", "delete"],
        url: str,
        json: dict[str, str] | None = None,
        headers: dict[str, str] | None = None,
        files: dict[str, str] | None = None,
        data: bytes | None = None,
    ) -> requests.Response:
        fun = getattr(self._session, method)
        response: requests.Response = fun(
            parse.urljoin(f"https://{self._hostname}", url),
            json=json,
            headers=headers,
            files=files,
            data=data,
        )
        if response.status_code != 200:
            _logger.error(
                "Request %s %s failed with status code %d and response %s",
                method,
                url,
                response.status_code,
                response.text,
            )
            raise ConnectionError(response.text)
        return response

    def _listen(self, callback: Callable[[str, list[str]], None], timeout: float):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        with connect(
            f"wss://{self._hostname}/desk/api/navigation/events",
            # server_hostname='robot.franka.de',
            ssl_context=ctx,
            additional_headers={"authorization": self._session.cookies.get("authorization")},  # type: ignore[arg-type]
        ) as websocket:
            self._listening = True
            while self._listening:
                try:
                    event: dict[str, bool] = json_module.loads(websocket.recv(timeout))
                    # detect button click event
                    pressed_buttons = []
                    for key, value in event.items():
                        if not value and value != self._button_states[key]:
                            # button click event detected
                            pressed_buttons.append(key)
                        self._button_states[key] = value
                    if len(pressed_buttons) > 0:
                        _logger.info("Buttons %s pressed", str(pressed_buttons))
                        callback(self._hostname, pressed_buttons)
                except TimeoutError:
                    pass

    def listen(self, callback: Callable[[str, list[str]], None]) -> None:
        """
        Starts a thread listening to Pilot button events. All the Pilot buttons,
        except for the `Pilot Mode` button can be captured. Make sure Pilot Mode is
        set to Desk instead of End-Effector to receive direction key events. You can
        change the Pilot mode by pressing the `Pilot Mode` button or changing the mode
        in the Desk. Events will be triggered while buttons are pressed down or released.

        Args:
          cb: Callback fucntion that is called whenever a button event is received from the
            Desk. The callback receives a dict argument that contains the triggered buttons
            as keys. The values of those keys will depend on the kind of event, either True
            for a button pressed down or False when released.
            The possible buttons are: `circle`, `cross`, `check`, `left`, `right`, `down`,
            and `up`.
        """
        self._listen_thread = threading.Thread(target=self._listen, args=(callback, 1.0))
        self._listen_thread.start()

    def stop_listen(self) -> None:
        """
        Stop listener thread (cf. :py:func:`panda_py.Desk.listen`).
        """
        self._listening = False
        if self._listen_thread is not None:
            self._listen_thread.join()


class FCI:
    """
    Can be used as a context manager to activate the Franka Control Interface (FCI).
    """

    def __init__(
        self, desk: Desk, unlock: bool = False, lock_when_done: bool = True, guiding_mode_when_done: bool = True
    ):
        self.desk = desk
        self.unlock = unlock
        self.lock_when_done = lock_when_done
        self.guiding_mode_when_done = guiding_mode_when_done

    def __enter__(self) -> Desk:
        self.desk.__enter__()
        if self.unlock:
            self.desk.unlock()
        self.desk.disable_guiding_mode()
        self.desk.activate_fci()
        return self.desk

    def __exit__(self, *args):
        self.desk.deactivate_fci()
        if self.lock_when_done:
            self.desk.lock()
        if self.guiding_mode_when_done:
            self.desk.enable_guiding_mode()

        self.desk.__exit__()


class GuidingMode:
    """
    Can be used as a context manager to enable or disable guiding mode.
    """

    def __init__(self, desk: Desk, unlock: bool = False):
        self.desk = desk
        self.unlock = unlock

    def __enter__(self) -> Desk:
        self.desk.__enter__()
        if self.unlock:
            self.desk.unlock()
        self.desk.activate_fci()
        self.desk.enable_guiding_mode()
        return self.desk

    def __exit__(self, *args):
        self.desk.disable_guiding_mode()
        self.desk.__exit__()
