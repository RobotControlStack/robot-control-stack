import logging
from contextlib import ExitStack
from pathlib import Path
from time import sleep
from typing import Annotated, Optional

import pyrealsense2 as rs
import rcsss
import rcsss.control.fr3_desk
import rcsss.control.server
import typer
from PIL import Image
from rcsss.camera.realsense import RealSenseCameraSet
from rcsss.config import create_sample_config_yaml, read_config_yaml
from rcsss.control.record import PoseList
from rcsss.control.utils import load_creds_fr3_desk
from rcsss.envs.factories import get_urdf_path

logger = logging.getLogger(__name__)

# MAIN CLI
main_app = typer.Typer(help="CLI tool for the Robot Control Stack (RCS).")


@main_app.command()
def sample_config(
    path: Annotated[
        str, typer.Option("-p", help="Path to where the default config file should be saved")
    ] = "config.yaml",
):
    """Creates a sample yaml config file"""
    create_sample_config_yaml(path)


# REALSENSE CLI
realsense_app = typer.Typer()
main_app.add_typer(
    realsense_app,
    name="realsense",
    help="Commands to access the intel realsense camera. This includes tools such as reading out the serial numbers of connected devices.",
)


@realsense_app.command()
def serials():
    """Reads out the serial numbers of the connected realsense devices."""
    context = rs.context()
    devices = RealSenseCameraSet.enumerate_connected_devices(context)
    if len(devices) == 0:
        logger.warning("No realsense devices connected.")
        return
    logger.info("Connected devices:")
    for device in devices.values():
        logger.info("  %s: %s", device.product_line, device.serial)


@realsense_app.command()
def test(
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Tests all configured and connected realsense by saving the current picture."""
    cfg = read_config_yaml(path)
    assert cfg.hw.camera_type == "realsense", "Only realsense cameras are supported for this test."
    assert cfg.hw.camera_config is not None, "Camera config is not set."
    assert cfg.hw.camera_config.realsense_config is not None, "Realsense config is not set."
    cs = RealSenseCameraSet(cfg.hw.camera_config.realsense_config)
    cs.warm_up()
    testdir = Path(cfg.hw.camera_config.realsense_config.record_path)
    frame_set = cs.poll_frame_set()
    for device_str in cfg.hw.camera_config.realsense_config.name_to_identifier:
        assert device_str in frame_set.frames
        frame = frame_set.frames[device_str]
        if frame.camera.color is not None:
            im = Image.fromarray(frame.camera.color.data, mode="RGB")
            im.save(testdir / f"test_img_{device_str}.png")
        if frame.camera.depth is not None:
            im = Image.fromarray(frame.camera.depth.data)
            im.save(testdir / f"test_depth_{device_str}.png")
        if frame.camera.ir is not None:
            im = Image.fromarray(frame.camera.ir.data)
            im.save(testdir / f"test_ir_{device_str}.png")
        if frame.imu is not None:
            logger.info("IMU data: %s %s", frame.imu.accel, frame.imu.gyro)


@realsense_app.command()
def test_record(
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    n_frames: Annotated[int, typer.Argument(help="Name of the camera that should be tested")] = 100,
):
    """Tests all configured and connected realsense by saving the current picture."""
    cfg = read_config_yaml(path)
    assert cfg.hw.camera_type == "realsense", "Only realsense cameras are supported for this test."
    assert cfg.hw.camera_config is not None, "Camera config is not set."
    assert cfg.hw.camera_config.realsense_config is not None, "Realsense config is not set."
    cs = RealSenseCameraSet(cfg.hw.camera_config.realsense_config)
    cs.start(warm_up=True)
    while cs.buffer_size() < n_frames:
        sleep(1 / cfg.hw.camera_config.realsense_config.frame_rate)
    cs.stop()


# FR3 CLI
fr3_app = typer.Typer()
main_app.add_typer(
    fr3_app,
    name="fr3",
    help="Commands to control a Franka Research 3. This includes tools that you would usually do with Franka's Desk interface.",
)


@fr3_app.command()
def home(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    shut: Annotated[bool, typer.Option("-s", help="Should the robot be shut down")] = False,
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
):
    """Moves the FR3 to home position"""
    user, pw = load_creds_fr3_desk()
    rcsss.control.fr3_desk.home(ip, user, pw, shut, unlock)


@fr3_app.command()
def info(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    include_gripper: Annotated[bool, typer.Option("-g", help="includes gripper")] = False,
):
    """Prints info about the robots current joint position and end effector pose, optionally also the gripper."""
    user, pw = load_creds_fr3_desk()
    rcsss.control.fr3_desk.info(ip, user, pw, include_gripper)

@fr3_app.command()
def run_server():
    """Starts the server for remote control."""
    from rcsss.envs.base import ControlMode, RelativeTo, RobotInstance
    rcsss.control.server.Server(robot_instance=RobotInstance.SIMULATION).start()

@fr3_app.command()
def lock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Locks the robot."""
    user, pw = load_creds_fr3_desk()
    rcsss.control.fr3_desk.lock(ip, user, pw)


@fr3_app.command()
def unlock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Prepares the robot by unlocking the joints and putting the robot into the FCI mode."""
    user, pw = load_creds_fr3_desk()
    rcsss.control.fr3_desk.unlock(ip, user, pw)
    with rcsss.control.fr3_desk.Desk(ip, user, pw) as d:
        d.activate_fci()


@fr3_app.command()
def fci(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
    shutdown: Annotated[bool, typer.Option("-s", help="After ctrl+c shuts the robot down")] = False,
):
    """Puts the robot into FCI mode, optionally unlocks the robot. Waits for ctrl+c to exit."""
    user, pw = load_creds_fr3_desk()
    try:
        with rcsss.control.fr3_desk.FCI(rcsss.control.fr3_desk.Desk(ip, user, pw), unlock=unlock, lock_when_done=False):
            while True:
                sleep(1)
    except KeyboardInterrupt:
        if shutdown:
            rcsss.control.fr3_desk.shutdown(ip, user, pw)


@fr3_app.command()
def guiding_mode(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    disable: Annotated[bool, typer.Option("-d", help="Disable guiding mode")] = False,
    unlock: Annotated[bool, typer.Option("-u", help="unlocks the robot")] = False,
):
    """Enables or disables guiding mode."""
    user, pw = load_creds_fr3_desk()
    rcsss.control.fr3_desk.guiding_mode(ip, user, pw, disable, unlock)


@fr3_app.command()
def shutdown(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
):
    """Shuts the robot down"""
    user, pw = load_creds_fr3_desk()
    rcsss.control.fr3_desk.shutdown(ip, user, pw)


@fr3_app.command()
def serve(
    port: Annotated[int, typer.Option(help="Port of the server")] = "18861",
    botip: Annotated[str, typer.Option(help="IP of the robot")] = "192.168.103.1",
):
    """Starts the server for remote control."""
    rcsss.control.server.Server(server_port=port, robot_ip=botip).start()


@fr3_app.command()
def record(
    ip_str: Annotated[str, typer.Argument(help="Name to IP dict. e.g. \"{'robot1': '192.168.100.1'}\"")],
    urdf_path: Annotated[Optional[str], typer.Option(help="Path to the urdf file")] = None,
    lpaths: Annotated[Optional[list[str]], typer.Option("--lpaths", help="Paths to load n recordings")] = None,
    spath: Annotated[Optional[str], typer.Option("--spath", help="Paths to load n recordings")] = None,
    buttons: Annotated[bool, typer.Option("-b", help="Use the robots buttons instead of the keyboard")] = False,
):
    """Tool to record poses with multiple FR3 robots."""
    user, pw = load_creds_fr3_desk()
    urdf_path = get_urdf_path(urdf_path, allow_none_if_not_found=False)  # type: ignore

    name2ip: dict[str, str] = eval(ip_str)

    if lpaths is not None and len(lpaths) > 0:
        with ExitStack() as stack:
            for r_ip in name2ip.values():
                stack.enter_context(rcsss.control.fr3_desk.Desk.fci(r_ip, username=user, password=pw, unlock=True))

            p = PoseList.load(name2ip, lpaths, urdf_path=urdf_path)
            input("Press any key to replay")
            p.replay()
    else:
        with ExitStack() as stack:
            gms = [
                rcsss.control.fr3_desk.Desk.guiding_mode(r_ip, username=user, password=pw, unlock=True)
                for r_ip in name2ip.values()
            ]
            for gm in gms:
                stack.enter_context(gm)
            p = PoseList(name2ip, urdf_path=urdf_path)
            if not buttons:
                p.record()
            else:
                p.start_button_recording()
                for gm in gms:
                    gm.desk.listen(p.button_callback)
                while p._button_recording:
                    pass
                for gm in gms:
                    gm.desk.stop_listen()

            if spath is not None:
                p.save(spath)


def main():
    main_app()
