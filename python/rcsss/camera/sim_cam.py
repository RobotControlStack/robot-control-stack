import mujoco as mj
import numpy as np
import glfw
from matplotlib import pyplot as plt

from rcsss import sim
from rcsss.camera.interface import (
    Camera,
    CameraFrame,
    Frame,
    GenericCameraConfig
)
# from rcsss import sim


class RealSenseConfig(GenericCameraConfig):
    pass


class SimCam(Camera):

    def __init__(self, model: mj.MjModel, state: mj.MjData, cfg: GenericCameraConfig, cam_name: str) -> None:
        self.model = model
        self.state = state
        self._cfg = cfg
        self.cam_name = cam_name

    def _setup_cam(self, cam_name):
        self.cam_id = self.model.camera(cam_name).id
        if self.cam_id == -1:
            self.cam.type = mj.mjtCamera.mjCAMERA_FREE
        else:
            self.cam.type = mj.mjtCamera.mjCAMERA_FIXED

        self.cam.fixedcamid = self.cam_id

    def _make_gl_context(self):
        glfw.init()
        self.width, self.height = glfw.get_video_mode(glfw.get_primary_monitor()).size
        glfw.window_hint(glfw.VISIBLE, 0)
        self.title = "rcsss_camera"
        self.window = glfw.create_window(self.width, self.height, self.title, None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

    def _setup_mjcontext(self):
        # create options, camera, scene, context
        self.vopt = mj.MjvOption()
        self.cam = mj.MjvCamera()
        self.scn = mj.MjvScene(self.model, maxgeom=10000)
        self.pert = mj.MjvPerturb()
        self.ctx = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

        # set_mujoco_buffer @todo setting to offscreen buffer causes unexpected image.
        # mj.mjr_setBuffer(mj.mjtFramebuffer.mjFB_OFFSCREEN, self.ctx)

    @property
    def config(self) -> GenericCameraConfig:
        return self._cfg

    def get_current_frame(self) -> Frame:
        # We must make GLContext before MjrContext
        self._make_gl_context()

        self._setup_mjcontext()
        self._setup_cam(self.cam_name)
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
        viewport = mj.mjr_maxViewport(self.ctx)
        mj.mjr_render(viewport, self.scn, self.ctx)
        shape = glfw.get_framebuffer_size(self.window)
        img = np.zeros((shape[1], shape[0], 3), dtype=np.uint8)
        mj.mjr_readPixels(img, None, viewport, self.ctx)
        img = np.flipud(img)
        camera_frame = CameraFrame(color=img, ir=None, depth=None, temperature=None)
        imu_frame = None
        # close glfw
        glfw.terminate()
        self.ctx.free()
        return Frame(camera=camera_frame, imu=imu_frame)


if __name__ == "__main__":
    """Here we can test the SimCam class as a stand-alone script."""
    def load_np_image(path: str):
        return np.load(path)

    # method 1 to get model and the state
    """
     @TODO: currently sim.FR3 works with a previous commit of models which doesnt have the camera names,
     so temporarily using method2
    """
    # robot = sim.FR3("models/mjcf/scene.xml", "models/urdf/fr3_from_panda.urdf", render=True)
    # model_ = robot.model
    # state_ = robot.data

    # method 2 to get the model and the state
    fr3_mjcf_path = 'models/mjcf/scene.xml'
    model_ = mj.MjModel.from_xml_path(filename=fr3_mjcf_path)
    state_ = mj.MjData(model_)
    # only needed for this standalone testing
    mj.mj_step(model_, state_)
    cam = SimCam(model_, state_, cfg=RealSenseConfig(), cam_name="eye-in-hand")
    frame = cam.get_current_frame()

    # the below code is just for visualising the captured frames
    img_ = frame.camera.color
    np.save('./viewer_read_pixels_testing.npy', img_)
    img_ = load_np_image("./viewer_read_pixels_testing.npy")
    plt.imshow(img_)
    plt.show()
