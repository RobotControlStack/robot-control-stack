import argparse
import rcsss

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Tool to move the FR3 to defined position."
    )
    parser.add_argument(
        "ip",
        type=str,
        help="IP of the robot"
    )
    args = parser.parse_args()

    f = rcsss.FR3(args.ip, "models/urdf/fr3.urdf")
    f.setParameters(0.7)
    g = rcsss.FrankaHand(args.ip)
    g.release()
    # g.shut()
    f.move_home()