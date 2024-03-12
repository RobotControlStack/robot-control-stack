import argparse
import rcss
import pw
import time

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

    f = rcss.Desk(args.ip, pw.username, pw.password)
    f.take_control(force=True)
    f.lock()
    f.shutdown()