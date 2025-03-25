import argparse
import cv2
from matplotlib import pyplot as plt
from rcsss.camera.interface import BaseCameraConfig
from rcsss.camera.realsense import RealSenseCameraSet, RealSenseSetConfig


if __name__ == "__main__":
    # arg parse to get the key for the camera
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=str, default="side")
    args = parser.parse_args()

    cameras = {
        "side2": BaseCameraConfig(identifier="244222071045"), # old wrist
        # "wrist": BaseCameraConfig(identifier="218622278131"), # new realsense
        "bird_eye": BaseCameraConfig(identifier="243522070364"), # bird eye
        # "side": BaseCameraConfig(identifier="243522070385"),
        "side": BaseCameraConfig(identifier="243522070385"), # side
    }

    cam_cfg = RealSenseSetConfig(
        cameras=cameras,
        resolution_width=1280,
        resolution_height=720,
        frame_rate=15,
        enable_imu=False,  # does not work with imu, why?
        enable_ir=True,
        enable_ir_emitter=False,
    )

    camera_set = RealSenseCameraSet(cam_cfg)
    frame_set = camera_set.poll_frame_set()

    # data = cv2.rotate(frame_set.frames[args.camera].camera.color.data, cv2.ROTATE_90_COUNTERCLOCKWISE)
    data = frame_set.frames[args.camera].camera.color.data
    img = plt.imshow(data)
    plt.show(block=False)
    plt.pause(0.1)

    # while True:
    for i in range(300):
        frame_set = camera_set.poll_frame_set()

        # # save the images in the frameset
        # for key, frame in frame_set.frames.items():
        #     cv2.imwrite(f"{key}.png", frame.camera.color.data)
        # break

        # img.set_data(frame_set.frames[args.camera].camera.color.data)
        # # data = cv2.rotate(frame_set.frames[args.camera].camera.color.data, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # plt.draw()
        # plt.pause(0.5)

    # save the images in the frameset
    for key, frame in frame_set.frames.items():
        # cv2.imwrite(f"{key}.png", frame.camera.color.data)
        # save in rgb
        cv2.imwrite(f"{key}.png", cv2.cvtColor(frame.camera.color.data, cv2.COLOR_BGR2RGB))
