import logging
from threading import Lock
from time import sleep

import numpy as np
from pynput import keyboard
from rcsss.control.fr3_desk import FCI, Desk
from rcsss.control.utils import load_creds_fr3_desk
from rcsss.envs.base import ControlMode, RelativeTo
from rcsss.envs.factories import fr3_hw_env
from rcsss.envs.utils import default_fr3_hw_gripper_cfg, default_fr3_hw_robot_cfg

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


ROBOT_IP = "192.168.101.1"
VALLID_KEYS = {"w", "a", "s", "d", "i", "k", "g"}
KEYMAP = {"w": (0, +1), "a": (1, -1), "s": (0, -1), "d": (1, +1), "i": (2, +1), "k": (2, -1)}


class RobotControl:
    def __init__(self, env):
        self._env = env
        self._lock = Lock()
        self._gripper_state = 1
        self._mode_rpy = False
        self._keys = set()
        self._exit = False

        self._tstep = 0.1
        self._astep = 0.1
        self._action_scale = 1.0
        self._scale_step = 0.1

    def on_press(self, key: keyboard.Key):
        if hasattr(key, "char") and key.char in VALLID_KEYS and key.char != "g":
            with self._lock:
                self._keys.add(key.char)

    def on_release(self, key: keyboard.Key):
        with self._lock:
            if key is keyboard.Key.alt_l:
                self._mode_rpy = not self._mode_rpy
                if self._mode_rpy:
                    print("Switching mode to RPY")
                else:
                    print("Switching mode to XYZ")
            elif key is keyboard.Key.page_up:
                self._action_scale = min(1.0, self._action_scale + self._scale_step)
                print(f"New speed factor: {self._action_scale:.2f}")
            elif key is keyboard.Key.page_down:
                self._action_scale = max(0.0, self._action_scale - self._scale_step)
                print(f"New speed factor: {self._action_scale:.2f}")
            elif key is keyboard.Key.esc:
                self._exit = True
                print("Exiting...")
            elif hasattr(key, "char") and key.char in VALLID_KEYS:
                if key.char == "g":
                    self._gripper_state = not self._gripper_state
                else:
                    self._keys.remove(key.char)

    def get_keyboard_action(self):
        action = np.zeros(7)
        done = False

        with self._lock:
            if not self._mode_rpy:
                for k in self._keys:
                    i, sgn = KEYMAP[k]
                    action[i] = sgn * self._tstep
            else:
                # Rotation
                offset = 3
                for k in self._keys:
                    i, sgn = KEYMAP[k]
                    action[offset + i] = sgn * self._astep

            action[:6] *= self._action_scale
            action[6] = self._gripper_state
            done = self._exit

        return action, done

    def step(self, action) -> tuple[bool, list[str], dict]:
        # TODO Check if the model indicates when an action is finished.
        obs, _, _, truncated, info = self._env.step({"xyzrpy": action[:6], "gripper": action[6]})
        return obs

    def loop(self):
        # Initialize the environment and obtain the initial observation
        _, _ = self._env.reset()

        sleep(5)
        _, _, _, truncated, info = self._env.step({"xyzrpy": np.array([0, 0, 0, 0, 0, 0]), "gripper": 1})

        print("Enter action (WASD: x/y plane, IK: z - ALT for mode switch - WASD: roll, pitch, IK: yaw - g: gripper)")
        done = False

        while not done:
            action, done = self.get_keyboard_action()
            _ = self.step(action)


def main():
    user, pw = load_creds_fr3_desk()
    d = Desk(ROBOT_IP, user, pw)
    with FCI(d, unlock=False, lock_when_done=False):
        env = fr3_hw_env(
            ROBOT_IP,
            control_mode=ControlMode.CARTESIAN_TRPY,
            robot_cfg=default_fr3_hw_robot_cfg(),
            collision_guard="lab",
            gripper_cfg=default_fr3_hw_gripper_cfg(),
            max_relative_movement=0.2,
            relative_to=RelativeTo.CONFIGURED_ORIGIN,
        )
        controller = RobotControl(env)
        listener = keyboard.Listener(on_press=controller.on_press, on_release=controller.on_release)
        listener.start()
        controller.loop()
        listener.stop()


if __name__ == "__main__":
    main()
