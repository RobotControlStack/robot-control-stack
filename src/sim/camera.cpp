
#include "camera.h"

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
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/UrdfFactory.h"

namespace rcs {
namespace sim {

SimCameraSet::SimCameraSet(std::shared_ptr<Sim> sim, const SimCameraConfig& cfg)
    : sim{sim}, cfg{cfg}, buffer{}, buffer_lock{} {
  this->sim->register_cb(std::bind(&SimCameraSet::frame_callback, this),
                         1.0 / this->cfg.frame_rate);
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

ColorFrame SimCameraSet::poll_frame(std::string camera_id) {
  // TODO: use this->sim to get the camera frame
}

void SimCameraSet::frame_callback() {
  FrameSet fs;
  for (auto const& [camera_id, _] : this->cfg.camera2id) {
    ColorFrame frame = poll_frame(camera_id);
    fs.color_frames[camera_id] = frame;
    // fs.timestamp = this->sim->get_time();
  }
  std::lock_guard<std::mutex> lock(buffer_lock);
  buffer.push_back(fs);
}

}  // namespace sim
}  // namespace rcs