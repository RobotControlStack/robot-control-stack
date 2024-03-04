import argparse
import rcsss
import pw
import time

def prepare(ip, guiding_mode=False):
    d = rcsss.Desk(ip, pw.username, pw.password)
    d.take_control(force=True)
    d.unlock()
    d.activate_fci()
    if guiding_mode:
        d.enable_guiding_mode()
    else:
        d.disable_guiding_mode()


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
    prepare(args.ip)
