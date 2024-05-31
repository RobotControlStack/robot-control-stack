import mujoco as mj
import numpy as np
import glfw
from matplotlib import pyplot as plt

from rcsss.camera.interface import (
    Camera,
    CameraFrame,
    Frame,
    GenericCameraConfig
)
# from rcsss import sim


class SimCamConfig(GenericCameraConfig):
    pass


class SimCam(Camera):
    """
    @TODO: Most of the code here is obtained from the following repo, need to check if there is any liabilities..
    https://github.com/rohanpsingh/mujoco-python-viewer/blob/main/mujoco_viewer/mujoco_viewer.py
    """

    def __init__(self, model: mj.MjModel, state: mj.MjData, cfg: SimCamConfig, cam_name: str) -> None:
        self.model = model
        self.state = state
        self._cfg = cfg
        self.cam_id = model.camera(cam_name).id
        # @TODO: verify if this is the right place to step, commenting the below line leads to unexpected images
        mj.mj_step(model, state)

    def _setup_glfw(self):
        glfw.init()
        self.width, self.height = glfw.get_video_mode(glfw.get_primary_monitor()).size
        glfw.window_hint(glfw.VISIBLE, 0)
        self.title = "rcsss_camera"
        self.window = glfw.create_window(self.width, self.height, self.title, None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        self.framebuffer_width, self.framebuffer_height = glfw.get_framebuffer_size(self.window)
        self.viewport = mj.MjrRect(0, 0, self.framebuffer_width, self.framebuffer_height)

    def _setup_cam(self):
        # create options, camera, scene, context
        self.vopt = mj.MjvOption()
        self.cam = mj.MjvCamera()
        self.scn = mj.MjvScene(self.model, maxgeom=10000)
        self.pert = mj.MjvPerturb()
        self.ctx = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)
        self.width, self.height = glfw.get_framebuffer_size(self.window)
        if self.cam_id == -1:
            self.cam.type = mj.mjtCamera.mjCAMERA_FREE
        else:
            self.cam.type = mj.mjtCamera.mjCAMERA_FIXED

    @property
    def config(self) -> SimCamConfig:
        return self._cfg

    @config.setter
    def config(self, cfg: SimCamConfig) -> None:
        self._cfg = cfg

    def get_current_frame(self) -> Frame:
        # initial setup
        self._setup_glfw()
        self._setup_cam()
        self.cam.fixedcamid = self.cam_id
        self.viewport.width, self.viewport.height = glfw.get_framebuffer_size(self.window)

        # update scene
        mj.mjv_updateScene(
            self.model,
            self.state,
            self.vopt,
            self.pert,
            self.cam,
            mj.mjtCatBit.mjCAT_ALL.value,
            self.scn)

        # render
        mj.mjr_render(self.viewport, self.scn, self.ctx)
        shape = glfw.get_framebuffer_size(self.window)
        img = np.zeros((shape[1], shape[0], 3), dtype=np.uint8)
        mj.mjr_readPixels(img, None, self.viewport, self.ctx)
        img = np.flipud(img)
        camera_frame = CameraFrame(color=img, ir=None, depth=None, temperature=None)
        imu_frame = None

        # close glfw
        glfw.terminate()
        self.ctx.free()
        return Frame(camera=camera_frame, imu=imu_frame)


# @TODO: remove this later
def load_np_image(path: str):
    return np.load(path)


if __name__ == "__main__":
    # @TODO: there is an error when using the model and state from the robot, need to check
    # robot = sim.FR3("models/mjcf/scene.xml", "models/urdf/fr3_from_panda.urdf", render=True)
    # model_ = robot.model
    # state_ = robot.data
    fr3_mjcf_path = 'models/mjcf/scene.xml'
    model_ = mj.MjModel.from_xml_path(filename=fr3_mjcf_path)
    state_ = mj.MjData(model_)
    cam = SimCam(model_, state_, cfg=SimCamConfig(), cam_name="eye-in-hand")
    frame = cam.get_current_frame()
    # the below code is just for visualising the captured frames
    img_ = frame.camera.color
    np.save('./viewer_read_pixels_testing.npy', img_)
    img_ = load_np_image("./viewer_read_pixels_testing.npy")
    plt.imshow(img_)
    plt.show()
