from typing import Protocol

import numpy as np


class BaseHand(Protocol):
    """
    Hand Class
    This class provides an interface for hand control.
    """

    def grasp(self):
        pass

    def open(self):
        pass

    def reset(self):
        pass

    def close(self):
        pass

    def get_state(self) -> np.ndarray:
        pass

    def get_normalized_joints_poses(self) -> np.ndarray:
        pass

    def set_normalized_joints_poses(self, values: np.ndarray):
        pass
