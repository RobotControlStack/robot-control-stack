
#include "sim/camera.h"

#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"
#include "rcs/Robot.h"

namespace rcs {
namespace sim {

SimCameraSet::SimCameraSet(
    std::shared_ptr<Sim> sim,
    std::unordered_map<std::string, SimCameraConfig> cameras_cfg,
    bool render_on_demand)
    : sim{sim},
      cameras_cfg{cameras_cfg},
      buffer{},
      buffer_lock{},
      cameras{},
      render_on_demand{render_on_demand} {
  for (auto const& [id, cam] : cameras_cfg) {
    this->sim->register_rendering_callback(
        [this](const std::string& id, mjrContext& ctx, mjvScene& scene,
               mjvOption& opt) { this->frame_callback(id, ctx, scene, opt); },
        id, cam.frame_rate, cam.resolution_width, cam.resolution_height);

    mjvCamera mjcam;
    mjv_defaultCamera(&mjcam);

    if (cam.type == CameraType::default_free) {
      mjv_defaultFreeCamera(this->sim->m, &mjcam);
    } else {
      mjcam.type = cam.type;
      mjcam.fixedcamid =
          mj_name2id(this->sim->m, mjOBJ_CAMERA, cam.identifier.c_str());
    }
    cameras[id] = mjcam;
  }
}

SimCameraSet::~SimCameraSet() {}
int SimCameraSet::buffer_size() {
  std::lock_guard<std::mutex> lock(buffer_lock);
  return buffer.size();
}
void SimCameraSet::clear_buffer() {
  std::lock_guard<std::mutex> lock(buffer_lock);
  // when we clear the buffer, there is no last image timestep
  this->last_ts = 0;
  buffer.clear();
}

std::optional<FrameSet> SimCameraSet::get_latest_frameset() {
  if (this->render_on_demand) {
    this->render_all();
  }
  if (buffer.empty()) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(buffer_lock);
  return buffer.back();
}
std::optional<FrameSet> SimCameraSet::get_timestamp_frameset(float ts) {
  std::lock_guard<std::mutex> lock(buffer_lock);
  for (auto it = buffer.rbegin(); it != buffer.rend(); ++it) {
    if (it->timestamp == ts) {
      return *it;
    }
  }
  return std::nullopt;
}

void SimCameraSet::render_all() {
  for (auto const& [id, cam] : this->cameras_cfg) {
    mjrContext* ctx = this->sim->renderer.get_context(id);
    this->render_single(id, *ctx, this->sim->renderer.scene,
                        this->sim->renderer.opt);
  }
}

void SimCameraSet::frame_callback(const std::string& id, mjrContext& ctx,
                                  mjvScene& scene, mjvOption& opt) {
  if (!this->render_on_demand) {
    this->render_single(id, ctx, scene, opt);
  }
}

void SimCameraSet::render_single(const std::string& id, mjrContext& ctx,
                                 mjvScene& scene, mjvOption& opt) {
  mjrRect viewport = mjr_maxViewport(&ctx);
  int W = viewport.width;
  int H = viewport.height;

  // allocate rgb and depth buffers
  ColorFrame frame = ColorFrame::Zero(3 * W * H);
  DepthFrame depth = DepthFrame::Zero(1 * W * H);
  // update abstract scene
  // TODO: we might be able to call this once for all cameras
  // there is also a mjv_updateCamera function
  mjv_updateScene(this->sim->m, this->sim->d, &opt, NULL, &this->cameras[id],
                  mjCAT_ALL, &scene);
  // mjv_updateCamera(this->sim->m, this->sim->d, &this->cameras[id], &scene);

  // render scene in offscreen buffer
  mjr_render(viewport, &scene, &ctx);

  // read rgb and depth buffers
  // depth documentation can be found here:
  // https://registry.khronos.org/OpenGL-Refpages/gl4/html/glReadPixels.xhtml
  mjr_readPixels(frame.data(), depth.data(), viewport, &ctx);

  auto ts = this->sim->d->time;
  std::lock_guard<std::mutex> lock(buffer_lock);
  // The following code assumes that all render callbacks for a timestep
  // happen directly after each other
  if (this->last_ts == ts) {
    buffer[buffer.size() - 1].color_frames[id] = frame;
    buffer[buffer.size() - 1].depth_frames[id] = depth;
  } else {
    FrameSet fs;
    fs.timestamp = ts;
    fs.color_frames[id] = frame;
    fs.depth_frames[id] = depth;
    buffer.push_back(fs);
    this->last_ts = ts;
  }
}

}  // namespace sim
}  // namespace rcs
