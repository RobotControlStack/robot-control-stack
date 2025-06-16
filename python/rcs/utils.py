import logging
from time import perf_counter, sleep

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class SimpleFrameRate:
    def __init__(self, frame_rate: int | float, loop_name: str = "SimpleFrameRate"):
        """SimpleFrameRate is a utility class to manage frame rates in a simple way.
        It allows you to call it in a loop, and it will sleep the necessary time to maintain the desired frame rate.

        Args:
            frame_rate (int | float): The desired frame rate in frames per second (int) or as a time interval in seconds (float).
        """
        self.t: float | None = None
        self._last_print: float | None = None
        self.frame_rate = frame_rate
        self.loop_name = loop_name

    def reset(self):
        self.t = None

    def __call__(self):
        if self.t is None:
            self.t = perf_counter()
            self._last_print = self.t
            sleep(1 / self.frame_rate if isinstance(self.frame_rate, int) else self.frame_rate)
            return
        sleep_time = (
            1 / self.frame_rate - (perf_counter() - self.t)
            if isinstance(self.frame_rate, int)
            else self.frame_rate - (perf_counter() - self.t)
        )
        if sleep_time > 0:
            sleep(sleep_time)
        if self._last_print is None or perf_counter() - self._last_print > 10:
            self._last_print = perf_counter()
            logger.info(f"FPS {self.loop_name}: {1 / (perf_counter() - self.t)}")

        self.t = perf_counter()
