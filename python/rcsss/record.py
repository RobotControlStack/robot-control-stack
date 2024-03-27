import logging
from abc import ABC, abstractmethod
from time import sleep
from typing import Dict, List, Optional, Tuple, cast

import numpy as np
from rcsss import hw

np.set_printoptions(precision=27)

_logger = logging.getLogger("record")


class Pose(ABC):

    @abstractmethod
    def replay(self, robot: Dict[str, hw.FR3], gripper: Dict[str, hw.FrankaHand]):
        pass

    @abstractmethod
    def __str__(self) -> str:
        pass

    @abstractmethod
    @classmethod
    def from_str(cls, line: str) -> "Pose":
        pass


class JointPose(Pose):
    def __init__(self, name: str, pose: Optional[np.ndarray] = None):
        self.pose = pose
        self.name = name

    def record(self, robot: Dict[str, hw.FR3]):
        self.pose = robot[self.name].get_joint_position()

    def replay(self, robot: Dict[str, hw.FR3], _: Dict[str, hw.FrankaHand]):
        if self.pose is not None:
            robot[self.name].set_joint_position(self.pose)

    def __str__(self) -> str:
        if self.pose is None:
            return f"JointPose;{self.name};None"
        ps = np.array2string(self.pose).replace("\n", "")
        return f"JointPose;{self.name};{ps}"

    @classmethod
    def from_str(cls, line: str) -> "Pose":
        l = line.replace("\n", "").split(";")
        return cls(l[1], np.fromstring(l[2].strip("[]"), sep=" "))


class GripperPose(Pose):
    SPEED = 0.1
    FORCE = 5
    EPSILON = 0.1

    def __init__(self, name: str, pose: Optional[float] = None):
        self.pose = pose
        self.name = name

    def record(self, shut: bool, gripper: Dict[str, hw.FrankaHand]):
        if shut:
            gripper[self.name].grasp()
            self.pose = 0.1  # gripper[self.name].getState()[1]
        else:
            gripper[self.name].release()
            self.pose = None

    def replay(self, _: Dict[str, hw.FR3], gripper: Dict[str, hw.FrankaHand]):
        if self.pose:
            config = hw.FHConfig()
            config.speed = self.SPEED
            config.force = self.FORCE
            config.grasping_width = self.pose
            config.epsilon_inner = self.EPSILON
            config.epsilon_outer = self.EPSILON

            gripper[self.name].set_parameters(config)
            gripper[self.name].grasp()
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

    def replay(self, *args):  # noqa: ARG002
        input("Press enter to continue")

    def __str__(self) -> str:
        return "WaitForInput"

    @classmethod
    def from_str(cls, line: str) -> Pose:  # noqa: ARG003
        return cls()


class WaitForDoubleTab(Pose):
    def __init__(self, name: str):
        self.name = name

    def record(self):
        pass

    def replay(self, robot: Dict[str, hw.FR3], _: Dict[str, hw.FrankaHand]):
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

    def replay(self, *args):  # noqa: ARG002
        sleep(self.t)

    def __str__(self) -> str:
        return f"Sleep;{self.t}"

    @classmethod
    def from_str(cls, line: str) -> Pose:
        l = line.replace("\n", "").split(";")
        return cls(float(l[1]))


def check_pose(pose: np.ndarray):
    if np.all(pose <= np.array([2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895])) and np.all(
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

    def replay(self, robot: Dict[str, hw.FR3], _):
        config: hw.FR3Config = cast(hw.FR3Config, robot[self.name].get_parameters())
        config.speed_factor = self.speed
        robot[self.name].set_parameters(config)

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
        self.r: Dict[str, hw.FR3] = {key: hw.FR3(ip, self.MODEL_PATH) for key, ip in ip.items()}
        self.g: Dict[str, hw.FrankaHand] = {key: hw.FrankaHand(ip) for key, ip in ip.items()}
        self.poses: List[Pose] = [ChnageSpeedFactor(speed_factor, key) for key in self.r] if poses is None else poses

        self.m: Dict[str, Tuple[str, np.ndarray]] = {}

    @classmethod
    def load(cls, ip: Dict[str, str], filenames: List[str]):
        poses = []
        for filename in filenames:

            def get_class(line: str) -> type[Pose]:
                pose_dict: Dict[str, type[Pose]] = {
                    "JointPose": JointPose,
                    "GripperPose": GripperPose,
                    "WaitForInput": WaitForInput,
                    "Sleep": Sleep,
                    "ChnageSpeedFactor": ChnageSpeedFactor,
                    "WaitForDoubleTab": WaitForDoubleTab,
                }
                first = line.split(";")[0].replace("\n", "")
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
                if not check_pose(self.r[i.split(" ")[1]].get_joint_position()):
                    _logger.warning("REJECTED due to joint constraints")
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
                w = WaitForInput()
                self.poses.append(w)
            elif i == "wd":
                wd = WaitForDoubleTab(name=i.split(" ")[1])
                self.poses.append(wd)
            elif i.split(" ")[0] == "sl":
                sl = Sleep(float(i.split(" ")[1]))
                self.poses.append(sl)
            elif i.split(" ")[0] == "sp":
                sp = ChnageSpeedFactor(name=i.split(" ")[1], speed=float(i.split(" ")[2]))
                self.poses.append(sp)
            elif i == "q":
                break
            elif i.split(" ")[0] == "re":
                # re robotname savename
                self.m[i.split(" ")[2]] = (
                    i.split(" ")[1],
                    self.r[i.split(" ")[1]].get_joint_position(),
                )
            elif i in self.m:
                j = JointPose(name=self.m[i][0], pose=self.m[i][1])
                self.poses.append(j)
            else:
                _logger.warning("Invalid input")

    def replay(self):
        for pose in self.poses:
            pose.replay(self.r, self.g)
