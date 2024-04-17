from typing import List

import rcsss
import rpyc
import rpyc.utils
import rpyc.utils.classic


@rpyc.service
class RCSService(rpyc.Service):
    def __init__(self, robots: List[rcsss.common.Robot]) -> None:
        super().__init__()
        self.robots = robots

    @rpyc.exposed
    def move_home(self, robots: List[int] = [0]):
        print("Server: Moving home... ")
        for r in robots:
            self.robots[r].move_home()

    @rpyc.exposed
    def get_cartesian_position(self, robots: List[int] = [0]) -> List[rcsss.common.Pose]:
        print("Server: Getting cartesian position... ")
        return [self.robots[r].get_cartesian_position() for r in robots]

    @rpyc.exposed
    def set_cartesian_position(self, targets: List[rcsss.common.Pose], robots: List[int] = [0]) -> None:
        print("Server: Setting cartesian position... ")
        for r in robots:
            self.robots[r].set_cartesian_position(rpyc.utils.classic.obtain(targets[0]))
