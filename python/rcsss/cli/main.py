from time import sleep
from contextlib import ExitStack
from pathlib import Path
from typing import Annotated, Optional

import rcsss
import typer
from rcsss.camera.realsense import RealSenseCameraSet
from rcsss.config import create_sample_config_yaml, read_config_yaml
from rcsss.record import PoseList
import pyrealsense2 as rs
from PIL import Image


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
        print("No realsense devices connected.")
        return
    print("Connected devices:")
    for device in devices.values():
        print(f"  {device.product_line}: {device.serial}")


@realsense_app.command()
def test(
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    # testdir: Annotated[Path, typer.Argument(help="Path to the folder where the test images should be saved")],
    # cam_name: Annotated[str, typer.Argument(help="Name of the camera that should be tested")],
):
    """Tests all configured and connected realsense by saving the current picture."""
    cfg = read_config_yaml(path)
    assert cfg.hw.camera_type == "realsense" and cfg.hw.camera_config.realsense_config is not None
    cs = RealSenseCameraSet(cfg.hw.camera_config.realsense_config)
    cs.warm_up()
    testdir = Path(cfg.hw.camera_config.realsense_config.record_path)
    frame_set = cs.poll_frame_set()
    for device_str in cfg.hw.camera_config.realsense_config.devices_to_enable:
        assert device_str in frame_set.frames
        frame = frame_set.frames[device_str]
        if frame.camera.depth is not None:
            im = Image.fromarray(frame.camera.color.data, mode="RGB")
            im.save(testdir / f"test_img_{device_str}.png")
        if frame.camera.depth is not None:
            im = Image.fromarray(frame.camera.depth.data)
            im.save(testdir / f"test_depth_{device_str}.png")
        if frame.camera.ir is not None:
            im = Image.fromarray(frame.camera.ir.data)
            im.save(testdir / f"test_ir_{device_str}.png")
        if frame.imu is not None:
            print("IMU data: ", frame.imu.accel, frame.imu.gyro)

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
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    shut: Annotated[bool, typer.Option("-s", help="Should the robot be shut down")] = False,
):
    """Moves the FR3 to home position"""
    cfg = read_config_yaml(path)
    rcsss.desk.home(ip, cfg.hw.username, cfg.hw.password, shut)


@fr3_app.command()
def lock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Locks the robot."""
    cfg = read_config_yaml(path)
    rcsss.desk.lock(ip, cfg.hw.username, cfg.hw.password)


@fr3_app.command()
def unlock(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Prepares the robot by unlocking the joints and putting the robot into the FCI mode."""
    cfg = read_config_yaml(path)
    rcsss.desk.unlock(ip, cfg.hw.username, cfg.hw.password)


@fr3_app.command()
def guiding_mode(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    disable: Annotated[bool, typer.Option("-d", help="Disable guiding mode")] = False,
):
    """Enables or disables guiding mode."""
    cfg = read_config_yaml(path)
    rcsss.desk.guiding_mode(ip, cfg.hw.username, cfg.hw.password, disable)


@fr3_app.command()
def shutdown(
    ip: Annotated[str, typer.Argument(help="IP of the robot")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
):
    """Shuts the robot down"""
    cfg = read_config_yaml(path)
    rcsss.desk.shutdown(ip, cfg.hw.username, cfg.hw.password)


@fr3_app.command()
def record(
    ip_str: Annotated[str, typer.Argument(help="Name to IP dict. e.g. \"{'robot1': '192.168.100.1'}\"")],
    path: Annotated[str, typer.Argument(help="Path to the config file")],
    lpaths: Annotated[Optional[list[str]], typer.Option("--lpaths", help="Paths to load n recordings")] = None,
    spath: Annotated[Optional[str], typer.Option("--spath", help="Paths to load n recordings")] = None,
    buttons: Annotated[bool, typer.Option("-b", help="Use the robots buttons instead of the keyboard")] = False,
):
    """Tool to record poses with multiple FR3 robots."""
    cfg = read_config_yaml(path)

    name2ip: dict[str, str] = eval(ip_str)

    if lpaths is not None and len(lpaths) > 0:
        with ExitStack() as stack:
            for r_ip in name2ip.values():
                stack.enter_context(
                    rcsss.desk.Desk.fci(r_ip, username=cfg.hw.username, password=cfg.hw.password, unlock=True)
                )

            p = PoseList.load(name2ip, lpaths, cfg.hw.urdf_model_path)
            input("Press any key to replay")
            p.replay()
    else:
        with ExitStack() as stack:
            gms = [
                rcsss.desk.Desk.guiding_mode(r_ip, username=cfg.hw.username, password=cfg.hw.password, unlock=True)
                for r_ip in name2ip.values()
            ]
            for gm in gms:
                stack.enter_context(gm)
            p = PoseList(name2ip, urdf_path=cfg.hw.urdf_model_path)
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
