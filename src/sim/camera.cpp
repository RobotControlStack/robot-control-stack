
#include "sim/camera.h"

#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
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
      [this](mjrContext& ctx, mjvScene& scene) {
        this->frame_callback(ctx, scene);
      },
      1.0 / this->cfg.frame_rate, this->cfg.resolution_width,
      this->cfg.resolution_height, false);
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

FrameSet SimCameraSet::get_latest_frameset() {
  std::lock_guard<std::mutex> lock(buffer_lock);
  return buffer.back();
}
FrameSet SimCameraSet::get_timestamp_frameset(float ts) {
  std::lock_guard<std::mutex> lock(buffer_lock);
  for (auto it = buffer.rbegin(); it != buffer.rend(); ++it) {
    if (it->timestamp == ts) {
      return *it;
    }
  }
  return FrameSet();
}

void SimCameraSet::frame_callback(mjrContext& ctx, mjvScene& scene) {
  FrameSet fs;
  for (auto const& [camera_id, _] : this->cfg.camera2id) {
    ColorFrame frame = poll_frame(camera_id, ctx, scene);
    fs.color_frames[camera_id] = frame;
  }
  fs.timestamp = this->sim->d->time;
  std::lock_guard<std::mutex> lock(buffer_lock);
  buffer.push_back(fs);
}

ColorFrame SimCameraSet::poll_frame(std::string camera_id, mjrContext& ctx,
                                    mjvScene& scene) {
  // TODO: use this->sim to get the camera frame
  // make sure the resulting ColorFrame memory (including its eigen matrix) is
  // owned by this class
  // TODO: Before we render the camera we might need to call mjv_updateScene
  return ColorFrame();
}

}  // namespace sim
}  // namespace rcs