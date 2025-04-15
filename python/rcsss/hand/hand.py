
class HandControl:
    """
    Base class for hand control.
    This class provides an interface for hand control, but does not implement any functionality.
    """
    def __init__(self):
        pass

    def grasp(self, value: float):
        raise NotImplementedError("This method should be overridden by subclasses.")

    def set_zero_pos(self):
        raise NotImplementedError("This method should be overridden by subclasses.")

    def disconnect(self):
        raise NotImplementedError("This method should be overridden by subclasses.")
        

class Hand:
    """
    Hand Class
    This class provides an interface for hand control.
    It allows for grasping, resetting, and disconnecting from the hand.
    """
    def __init__(self, hand: HandControl):
        self._hand = hand

    def grasp(self):
        self._hand.grasp(value = 0.9)

    def reset(self):
        self._hand.set_zero_pos()

    def get_state(self):
        pass
    
    def disconnect(self):
        self._hand.disconnect()

    def get_normalized_joints_poses(self):
        self._hand.get_pos_vector()

    def set_normalized_joints_poses(self, values: list):
        self._hand.set_pos_vector(values)    

    def open(self):
        self.reset()

    def __del__(self):
        self._hand.disconnect()