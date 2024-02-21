
import argparse
import pyfr3
import pw
import time

def lock(ip, guiding_mode=False):
    d = pyfr3.Desk(ip, pw.username, pw.password)
    d.take_control(force=True)
    d.lock()

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
    lock(args.ip)