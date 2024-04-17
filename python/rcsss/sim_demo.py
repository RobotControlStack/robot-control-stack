import numpy as np
import rcsss

MODEL_DIR = "../../models/"
iso_cube_center = np.array([0.498, 0.0, 0.226])
iso_cube_size = 0.4


def random_rpy():
    return rcsss.common.RPY(0, np.pi, np.random.uniform(-np.pi, np.pi))


def random_point_in_iso_cube():
    return np.random.uniform(iso_cube_center - iso_cube_size / 2, iso_cube_center + iso_cube_size / 2)


if __name__ == "__main__":
    robot = rcsss.sim.FR3(MODEL_DIR + "/mjcf/scene.xml", MODEL_DIR + "/urdf/fr3_from_panda.urdf", True)
    for i in range(100):
        print("Setting cartesian position... ")
        target = rcsss.common.Pose(random_rpy(), random_point_in_iso_cube())
        robot.set_cartesian_position(target)
        print("Done... ")
        print("Moving home... ")
        robot.move_home()
        print("Done... ")

    print("Exiting")
