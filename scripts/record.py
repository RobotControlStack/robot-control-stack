from abc import ABC
import argparse
from collections import defaultdict
from typing import Dict, Optional, List, Tuple, Union
import pyfr3
import numpy as np
from time import sleep
import pickle
import prepare

np.set_printoptions(precision=27)


class Pose(ABC):
    def record(self):
        pass

    def replay(self):
        pass

    def __str__(self) -> str:
        pass

    @classmethod
    def from_str(cls, line: str) -> "Pose":
        pass


class JointPose(Pose):
    def __init__(self, name: str, pose: Optional[np.ndarray] = None):
        self.pose = pose
        self.name = name

    def record(self, robot: Dict[str, pyfr3.FR3]):
        self.pose = robot[self.name].getJointPosition()

    def replay(self, robot: Dict[str, pyfr3.FR3], _: Dict[str, pyfr3.FrankaHand]):
        robot[self.name].setJointPosition(self.pose)

    def __str__(self) -> str:
        ps = np.array2string(self.pose).replace("\n", "")
        return f"JointPose;{self.name};{ps}"

    @classmethod
    def from_str(cls, line: str) -> "Pose":
        l = line.replace("\n", "").split(";")
        return cls(l[1], np.fromstring(l[2].strip("[]"), sep=" "))


class GripperPose(Pose):
    SPEED = 0.1
    FORCE = 5

    def __init__(self, name: str, pose: Optional[float] = None):
        self.pose = pose
        self.name = name

    def record(self, shut: bool, gripper: Dict[str, pyfr3.FrankaHand]):
        if shut:
            gripper[self.name].halt()
            self.pose = gripper[self.name].getState()[1]
        else:
            gripper[self.name].release()
            self.pose = None

    def replay(self, _: Dict[str, pyfr3.FR3], gripper: Dict[str, pyfr3.FrankaHand]):
        if self.pose:
            gripper[self.name].setParameters(self.pose, self.SPEED, self.FORCE)
            gripper[self.name].halt()
        else:
            gripper[self.name].release()

    def __str__(self) -> str:
        return f"GripperPose;{self.name};{self.pose}"

    @classmethod
    def from_str(cls, line: str) -> Pose:
        l = line.replace("\n", "").split(";")
        return cls(l[1], float(l[2]) if l[2] != "None" else None)


class WaitForInput(Pose):
    def __init__(self):
        pass

    def record(self):
        pass

    def replay(self, *args):
        input("Press enter to continue")

    def __str__(self) -> str:
        return "WaitForInput"

    @classmethod
    def from_str(cls, line: str) -> Pose:
        l = line.replace("\n", "").split(";")
        return cls()


class WaitForDoubleTab(Pose):
    def __init__(self, name: str):
        self.name = name

    def record(self):
        pass

    def replay(self, robot: Dict[str, pyfr3.FR3], _: Dict[str, pyfr3.FrankaHand]):
        robot[self.name].double_tap_robot_to_continue()

    def __str__(self) -> str:
        return f"WaitForDoubleTab;{self.name}"

    @classmethod
    def from_str(cls, line: str) -> "Pose":
        l = line.replace("\n", "").split(";")
        return cls(l[1])


class Sleep(Pose):
    def __init__(self, t):
        self.t = t

    def record(self):
        pass

    def replay(self, *args):
        sleep(self.t)

    def __str__(self) -> str:
        return f"Sleep;{self.t}"

    @classmethod
    def from_str(cls, line: str) -> Pose:
        l = line.replace("\n", "").split(";")
        return cls(float(l[1]))


def check_pose(pose: np.ndarray):
    if np.all(
        pose <= np.array([2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895])
    ) and np.all(
        np.array([-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895]) <= pose
    ):
        return True
    return False


class ChnageSpeedFactor(Pose):
    def __init__(self, speed: float, name: str):
        self.speed = min(max(0, speed), 1)
        self.name = name

    def record(self):
        pass

    def replay(self, robot, _):
        robot[self.name].setParameters(self.speed)

    def __str__(self) -> str:
        return f"ChnageSpeedFactor;{self.name};{self.speed}"

    @classmethod
    def from_str(cls, line: str) -> Pose:
        l = line.replace("\n", "").split(";")
        return cls(float(l[2]), l[1])


class PoseList:
    # should record and replay a list of poses
    MODEL_PATH = "models/urdf/fr3.urdf"

    def __init__(
        self,
        ip: Dict[str, str],
        speed_factor: float = 0.2,
        poses: Optional[List[Pose]] = None,
    ):
        self.r: Dict[str, pyfr3.FR3] = {
            key: pyfr3.FR3(ip, self.MODEL_PATH) for key, ip in ip.items()
        }
        self.g: Dict[str, pyfr3.FR3] = {
            key: pyfr3.FrankaHand(ip) for key, ip in ip.items()
        }
        self.poses: List[Pose] = (
            [ChnageSpeedFactor(speed_factor, key) for key in self.r]
            if poses is None
            else poses
        )

        self.m = {}

    @classmethod
    def load(cls, ip: Dict[str, str], filenames: List[str]):
        poses = []
        for filename in filenames:
            pose_dict = {
                "JointPose": JointPose,
                "GripperPose": GripperPose,
                "WaitForInput": WaitForInput,
                "Sleep": Sleep,
                "ChnageSpeedFactor": ChnageSpeedFactor,
                "WaitForDoubleTab": WaitForDoubleTab,
            }

            def get_class(line: str) -> Pose:
                first = line.split(";")[0].replace("\n", "")
                print(line)
                return pose_dict[first]

            with open(filename, "r") as f:
                poses += [get_class(line).from_str(line) for line in f.readlines()]

        return cls(poses=poses, ip=ip)

    def save(self, filename):
        with open(filename, "w") as f:
            f.write("\n".join([str(pose) for pose in self.poses]))

    def record(self):
        while True:
            i = input(
                "Press p to record a pose, press s to shut the gripper, press r to release the gripper, press w to have a wait for input pose\n"
            )
            if i.split(" ")[0] == "p":
                if not check_pose(self.r[i.split(" ")[1]].getJointPosition()):
                    print("REJECTED due to joint constraints")
                    continue
                j = JointPose(name=i.split(" ")[1])
                j.record(self.r)
                self.poses.append(j)
            elif i.split(" ")[0] == "s":
                g = GripperPose(name=i.split(" ")[1])
                g.record(True, self.g)
                self.poses.append(g)
            elif i.split(" ")[0] == "r":
                g = GripperPose(name=i.split(" ")[1])
                g.record(False, self.g)
                self.poses.append(g)
            elif i == "w":
                p = WaitForInput()
                self.poses.append(p)
            elif i == "w":
                p = WaitForDoubleTab(name=i.split(" ")[1])
                self.poses.append(p)
            elif i.split(" ")[0] == "sl":
                p = Sleep(float(i.split(" ")[1]))
                self.poses.append(p)
            elif i.split(" ")[0] == "sp":
                p = ChnageSpeedFactor(
                    name=i.split(" ")[1], speed=float(i.split(" ")[2])
                )
                self.poses.append(p)
            elif i == "q":
                break
            elif i.split(" ")[0] == "re":
                # re robotname savename
                self.m[i.split(" ")[2]] = (
                    i.split(" ")[1],
                    self.r[i.split(" ")[1]].getJointPosition(),
                )
            else:
                if i in self.m:
                    j = JointPose(name=self.m[i][0], pose=self.m[i][1])
                    self.poses.append(j)
                else:
                    print("Invalid input")

    def replay(self):
        for pose in self.poses:
            pose.replay(self.r, self.g)
            # sleep(0.1)


record = False
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tool to record poses with FR3.")
    parser.add_argument(
        "ip", type=str, help="Name to IP dict. e.g. \"{'robot1': '192.168.100.1'}\""
    )
    parser.add_argument(
        "--lpaths", type=str, nargs="+", help="Paths to load n recordings", default=[]
    )
    parser.add_argument(
        "--spath", type=str, help="Path to store the recoding", default=None
    )
    args = parser.parse_args()
    ip = eval(args.ip)

    if len(args.lpaths) != 0:
        for robot, r_ip in ip.items():
            prepare.prepare(r_ip, guiding_mode=False)
        p = PoseList.load(ip, args.lpaths)
        input("Press any key to replay")
        p.replay()
    else:
        for robot, ip in ip.items():
            prepare.prepare(ip, guiding_mode=True)
        p = PoseList(ip)
        p.record()
        if args.spath:
            p.save(args.spath)
