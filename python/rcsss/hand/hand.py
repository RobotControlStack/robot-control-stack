
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

    def get_normalized_width(self):
        pass

    def open(self):
        self.reset()

    def __del__(self):
        self._hand.disconnect()

if __name__ == "__main__":
    # try:
    #     hand = TilburgHandControl(verbose=True)
    #     hand.grasp(0.9)
    #     logger.info(f"current joints poses: {hand.get_pos_vector()}")
    #     logger.info(hand.pos_vector_names_list)
    #     hand.sleep(5)
    #     hand.set_zero_pos()
    #     hand.sleep(1)
    #     hand.set_joint_pos(hand.fingers_joints.THUMB_CMC, 0.5)
    #     hand.sleep(2)
    #     logger.info(f"current joint {hand.fingers_joints.THUMB_CMC.name}: {hand.get_pos_single(hand.fingers_joints.THUMB_CMC)}")
    #     hand.reset_joint_pos(hand.fingers_joints.THUMB_CMC)
    # except Exception as e:
    #     logger.exception(f"An error occurred: {e}")
    # finally:
    #     hand.disconnect()
    hand = Hand(TilburgHandControl(verbose=True))
    hand.reset()
    hand.grasp()
    sleep(5)
    hand.reset()
    sleep(2)
    hand.disconnect()