
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

#include "common/Robot.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"

namespace rcs {
namespace sim {

SimCameraSet::SimCameraSet(std::shared_ptr<Sim> sim, SimCameraSetConfig cfg)
    : sim{sim}, cfg{cfg}, buffer{}, buffer_lock{}, cameras{} {
  for (auto const& [id, cam] : cfg.cameras) {
    this->sim->register_rendering_callback(
        [this](const std::string& id, mjrContext& ctx, mjvScene& scene,
               mjvOption& opt) { this->frame_callback(id, ctx, scene, opt); },
        id, 1.0 / this->cfg.frame_rate, this->cfg.resolution_width,
        this->cfg.resolution_height, !cam.on_screen_render);

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
  buffer.clear();
}

std::optional<FrameSet> SimCameraSet::get_latest_frameset() {
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

void SimCameraSet::frame_callback(const std::string& id, mjrContext& ctx,
                                  mjvScene& scene, mjvOption& opt) {
  mjrRect viewport = mjr_maxViewport(&ctx);
  int W = viewport.width;
  int H = viewport.height;

  // allocate rgb buffers
  ColorFrame frame = ColorFrame::Zero(3 * W * H);

  // update abstract scene
  // TODO: we might be able to call this once for all cameras
  // there is also a mjv_updateCamera function
  mjv_updateScene(this->sim->m, this->sim->d, &opt, NULL, &this->cameras[id],
                  mjCAT_ALL, &scene);
  // mjv_updateCamera(this->sim->m, this->sim->d, &this->cameras[id], &scene);

  // render scene in offscreen buffer
  mjr_render(viewport, &scene, &ctx);

  // read rgb and depth buffers
  mjr_readPixels(frame.data(), NULL, viewport, &ctx);

  auto ts = this->sim->d->time;
  std::lock_guard<std::mutex> lock(buffer_lock);
  // The following code assumes that all render callbacks for a timestep
  // happen directly after each other
  if (this->last_ts == ts) {
    buffer[buffer.size() - 1].color_frames[id] = frame;
  } else {
    FrameSet fs;
    fs.timestamp = ts;
    fs.color_frames[id] = frame;
    buffer.push_back(fs);
    this->last_ts = ts;
  }
}

}  // namespace sim
}  // namespace rcs