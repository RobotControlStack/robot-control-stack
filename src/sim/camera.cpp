
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

SimCameraSet::SimCameraSet(std::shared_ptr<Sim> sim, const SimCameraConfig& cfg)
    : sim{sim}, cfg{cfg}, buffer{}, buffer_lock{} {
  this->sim->register_rendering_callback(
      [this](mjrContext& ctx, mjvScene& scene, mjvOption& opt) {
        this->frame_callback(ctx, scene, opt);
      },
      1.0 / this->cfg.frame_rate, this->cfg.resolution_width,
      this->cfg.resolution_height, true);  // offscreen = true
}

SimCameraSet::~SimCameraSet() {}
int SimCameraSet::buffer_size() {
  std::lock_guard<std::mutex> lock(buffer_lock);
  return buffer.size();
}
void SimCameraSet::clear_buffer() {
  std::lock_guard<std::mutex> lock(buffer_lock);
  // TODO: free from buffer elements
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

void SimCameraSet::frame_callback(mjrContext& ctx, mjvScene& scene, mjvOption& opt) {
  FrameSet fs;
  for (auto const& [cameraname, mjcfname] : this->cfg.camera2mjcfname) {
    ColorFrame frame = poll_frame(mjcfname, ctx, scene, opt);
    fs.color_frames[cameraname] = frame;
  }
  fs.timestamp = this->sim->d->time;
  std::lock_guard<std::mutex> lock(buffer_lock);
  buffer.push_back(fs);
}

ColorFrame SimCameraSet::poll_frame(std::string camera_id, mjrContext& ctx,
                                    mjvScene& scene, mjvOption& opt) {
  // TODO: manage the camera in attribute vector
  mjvCamera cam;

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  // TODO: default free camera as camera type
  // mjv_defaultFreeCamera(model, &cam);
  mjv_defaultOption(&opt);

  cam.type = mjCAMERA_FIXED;
  cam.fixedcamid = mj_name2id(this->sim->m, mjOBJ_CAMERA, camera_id.c_str());

  mjrRect viewport = mjr_maxViewport(&ctx);
  int W = viewport.width;
  int H = viewport.height;

  // allocate rgb buffers
  ColorFrame frame = ColorFrame::Zero(3 * W * H);

  // update abstract scene
  // TODO: we might be able to call this once for all cameras
  // there is also a mjv_updateCamera function
  mjv_updateScene(this->sim->m, this->sim->d, &opt, NULL, &cam, mjCAT_ALL,
                  &scene);

  // render scene in offscreen buffer
  mjr_render(viewport, &scene, &ctx);

  // read rgb and depth buffers
  mjr_readPixels(frame.data(), NULL, viewport, &ctx);

  return frame;
}

}  // namespace sim
}  // namespace rcs