import numpy as np
import rcsss
import rpyc

MODEL_DIR = "/home/xubuntu/Code/robot-control-stack/models"
iso_cube_center = np.array([0.498, 0.0, 0.226])
iso_cube_size = 0.4


def random_rpy():
    return rcsss.common.RPY(0, np.pi, np.random.uniform(-np.pi, np.pi))


def random_point_in_iso_cube():
    return np.random.uniform(iso_cube_center - iso_cube_size / 2, iso_cube_center + iso_cube_size / 2)


if __name__ == "__main__":
    c = rpyc.connect("localhost", 18861, config= {"allow_pickle": True})
    for i in range(100):
        print("Client: Setting cartesian position... ")
        # Generate random pose
        target = rcsss.common.Pose(random_rpy(), random_point_in_iso_cube())
        c.root.set_cartesian_position([target])
        print("Client: Done... ")
        print("Client: Moving home... ")
        c.root.move_home()
        print("Client: Done... ")

    print("Client: Exiting")
