import threading
import time

import evdev
from evdev import ecodes


class FootPedal:
    def __init__(self, device_name_substring="Foot Switch"):
        """Initializes the foot pedal and starts the background reading thread."""
        self.device_path = self._find_device(device_name_substring)

        if not self.device_path:
            msg = f"Could not find a device matching '{device_name_substring}'"
            raise FileNotFoundError(msg)

        self.device = evdev.InputDevice(self.device_path)
        self.device.grab()  # Prevent events from leaking into the OS/terminal

        # Dictionary to hold the current state of each key.
        # True = Pressed/Held, False = Released
        self._key_states = {}
        self._lock = threading.Lock()

        # Start the background thread
        self._running = True
        self._thread = threading.Thread(target=self._read_events, daemon=True)
        self._thread.start()
        print(f"Connected to {self.device.name} at {self.device_path}")

    def _find_device(self, substring):
        """Finds the device path for the foot pedal."""
        for path in evdev.list_devices():
            dev = evdev.InputDevice(path)
            if substring.lower() in dev.name.lower():
                return path
        return None

    def _read_events(self):
        """Background loop that updates the state dictionary."""
        try:
            for event in self.device.read_loop():
                if not self._running:
                    break

                if event.type == ecodes.EV_KEY:
                    key_event = evdev.categorize(event)

                    if isinstance(key_event, evdev.KeyEvent):
                        with self._lock:
                            # keystate: 1 is DOWN, 2 is HOLD, 0 is UP
                            is_pressed = key_event.keystate in [1, 2]

                            # Store state using the string name of the key (e.g., 'KEY_A')
                            # If a key resolves to a list (rare, but happens in evdev), take the first one
                            key_name = key_event.keycode
                            if isinstance(key_name, list):
                                key_name = key_name[0]

                            self._key_states[key_name] = is_pressed

        except OSError:
            pass  # Device disconnected or closed

    def get_states(self):
        """
        Returns a snapshot of the latest key states.
        Example return: {'KEY_A': True, 'KEY_B': False, 'KEY_C': False}
        """
        with self._lock:
            # Return a copy to ensure thread safety
            return self._key_states.copy()

    def get_key_state(self, key_name):
        """Returns the state of a specific key, defaulting to False if never pressed."""
        with self._lock:
            return self._key_states.get(key_name, False)

    def close(self):
        """Cleans up the device and stops the thread."""
        self._running = False
        try:
            self.device.ungrab()
            self.device.close()
        except OSError:
            pass


# ==========================================
# Example Usage
# ==========================================
if __name__ == "__main__":
    try:
        # Initialize the pedal
        pedal = FootPedal("Foot Switch")

        # Simulate a typical robotics control loop running at 10Hz
        print("Starting control loop... Press Ctrl+C to exit.")
        while True:
            # Grab the latest states instantly without blocking
            states = pedal.get_states()

            if states:
                # Print only the keys that are currently pressed
                pressed_keys = [key for key, is_pressed in states.items() if is_pressed]
                print(f"Currently pressed: {pressed_keys}")

            # Your teleoperation logic goes here...

            time.sleep(0.1)  # 10Hz loop

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if "pedal" in locals():
            pedal.close()
