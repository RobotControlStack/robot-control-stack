from abc import ABC
from typing import Optional, List, Tuple
import pyfr3
import numpy as np
from time import sleep
import pickle

ip = "192.168.101.1"
# model_path = "models/panda.urdf"
# poses_path = "scripts/poses.npy"


class Pose(ABC):
    def record(self):
        pass
    def replay(self):
        pass

class JointPose(Pose):
    def __init__(self, pose: Optional[np.ndarray] = None):
        self.pose = pose

    def record(self, robot: pyfr3.FR3):
        self.pose = robot.getJointPosition()

    def replay(self, robot: pyfr3.FR3, _: pyfr3.FrankaHand):
        robot.setJointPosition(self.pose)


class GripperPose(Pose):
    SPEED = 0.1
    FORCE = 5
    def __init__(self, shut: Optional[float] = None):
        self.shut = shut

    def record(self, shut: bool, gripper: pyfr3.FrankaHand):
        if shut:
            gripper.shut()
            self.pose = gripper.getState()[1]
        else:
            gripper.release()
            self.pose = None

    def replay(self, _: pyfr3.FR3, gripper: pyfr3.FrankaHand):
        if self.pose:
            gripper.setParameters(self.pose, self.SPEED, self.FORCE)
            gripper.shut()
        else:
            gripper.release()

class WaitForInput(Pose):
    def __init__(self):
        pass

    def record(self):
        pass

    def replay(self, *args):
        input("Press enter to continue")

class Sleep(Pose):
    def __init__(self, t):
        self.t = t

    def record(self):
        pass

    def replay(self, *args):
        sleep(self.t)

class ChnageSpeedFactor(Pose):
    def __init__(self, s):
        self.t = s

    def record(self):
        pass

    def replay(self, robot, _):
        robot.setParameters(self.t)

class PoseList:
    # should record and replay a list of poses
    MODEL_PATH = "models/panda.urdf"
    def __init__(self, ip, filename: str, load: bool = False):
        self.poses: List[Pose] = []
        self.filename = filename
        if load:
            self.load()
        self.r = pyfr3.FR3(ip, self.MODEL_PATH)
        self.r.setParameters(0.5)
        self.g = pyfr3.FrankaHand(ip)
        self.m = {}

    def load(self):
        # load with pickle
        with open(self.filename, "rb") as f:    
            self.poses = pickle.load(f)

    def save(self):
        # save with pickle
        with open(self.filename, "wb") as f:
            pickle.dump(self.poses, f)


    def record(self):
        while True:
            # try:
            i = input("Press p to record a pose, press s to shut the gripper, press r to release the gripper, press w to have a wait for input pose\n")
            if i == "p":
                j = JointPose()
                j.record(self.r)
                self.poses.append(j)
            elif i == "s":
                g = GripperPose()
                g.record(True, self.g)
                self.poses.append(g)
            elif i == "r":
                g = GripperPose()
                g.record(False, self.g)
                self.poses.append(g)
            elif i == "w":
                p = WaitForInput()
                self.poses.append(p)
            elif i.split(" ")[0] == "sl":
                p = Sleep(float(i.split(" ")[1]))
                self.poses.append(p)
            elif i.split(" ")[0] == "sp":
                p = Sleep(float(i.split(" ")[1]))
                self.poses.append(p)
            elif i == "q":
                break
            elif i.split(" ")[0] == "re":
                self.m[i.split(" ")[1]] = self.r.getJointPosition()
            else:
                if i in self.m:
                    j = JointPose(self.m[i])
                    self.poses.append(j)
                else:
                    print("Invalid input")

            # except KeyboardInterrupt:
            #     break
        self.save()

    def replay(self):
        for pose in self.poses:
            pose.replay(self.r, self.g)
            # if isinstance(pose, JointPose):
            #     pose.replay(self.r, None)
            # else:
            # sleep(0.1)




# def record_poses(f: pyfr3.FR3):
#     poses = []
#     while True:
#         try:
#             input("Press p to record a pose, press s to shut the gripper, press r to release the gripper")
#             pose = f.getJointPosition()
#             print(pose)
#             poses.append(pose)
#         except KeyboardInterrupt:
#             break
#     return poses

# def replay_poses(f: pyfr3.FR3, poses: list):
#     for pose in poses:
#         f.setJointPosition(pose)
#         # input("Press enter to continue")
#         sleep(0.1)

# def write_poses(poses: list, filename: str):
#     np.save(filename, poses)

# def read_poses(filename: str):
#     return np.load(filename)
    

record = True
if __name__ == "__main__":
    # f = pyfr3.FR3(ip, model_path)
    # g = pyfr3.FrankaHand(ip)
    # g.setParameters(0.072, 0.1, 5)
    # if record:
    #     poses = record_poses(f)
    #     write_poses(poses, poses_path)
    # else:
    #     input("Press enter to record a pose")
    #     replay_poses(f, read_poses(poses_path))
    p = PoseList(ip, "scripts/wave_better2.pickle", load=not record)
    if record:
        p.record()
    input("Press enter to replay")
    p.replay()

