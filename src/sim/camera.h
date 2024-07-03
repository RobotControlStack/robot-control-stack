#ifndef RCS_CAMSIM_H
#define RCS_CAMSIM_H
#include <common/Pose.h>
#include <common/Robot.h>
#include <mujoco/mujoco.h>

#include <eigen3/Eigen/Eigen>
#include <mutex>
#include <optional>
#include <set>
#include <unordered_map>

#include "sim/sim.h"

namespace rcs {
namespace sim {

enum CameraType {
  free = mjCAMERA_FREE,
  tracking = mjCAMERA_TRACKING,
  fixed = mjCAMERA_FIXED,
  default_free
};
struct SimCameraConfig {
  std::string identifier;
  CameraType type;
  bool on_screen_render;
};
struct SimCameraSetConfig {
  std::unordered_map<std::string, SimCameraConfig> cameras;
  int frame_rate;
  int resolution_width;
  int resolution_height;
};

// (H,W,3)
typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> ColorFrame;

struct FrameSet {
  std::unordered_map<std::string, ColorFrame> color_frames;
  mjtNum timestamp;
};

class SimCameraSet {
 public:
  SimCameraSet(std::shared_ptr<rcs::sim::Sim> sim, SimCameraSetConfig cfg);
  ~SimCameraSet();

  int buffer_size();
  void clear_buffer();

  std::optional<FrameSet> get_latest_frameset();
  std::optional<FrameSet> get_timestamp_frameset(float ts);

  void frame_callback(const std::string& id, mjrContext& ctx, mjvScene& scene,
                      mjvOption& opt);

 private:
  const SimCameraSetConfig cfg;
  std::shared_ptr<Sim> sim;
  std::vector<FrameSet> buffer;
  std::unordered_map<std::string, mjvCamera> cameras;
  std::mutex buffer_lock;
  mjtNum last_ts = 0;
};
}  // namespace sim
}  // namespace rcs
#endif  // RCS_CAMSIM_H