import argparse
import pyfr3

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

    f = pyfr3.FR3(args.ip, "models/urdf/fr3.urdf")
    g = pyfr3.FrankaHand(args.ip)
    g.shut()
    f.move_home()