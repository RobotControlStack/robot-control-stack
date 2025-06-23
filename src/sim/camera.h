#ifndef RCS_CAMSIM_H
#define RCS_CAMSIM_H
#include <mujoco/mujoco.h>
#include <rcs/Pose.h>
#include <rcs/Robot.h>

#include <eigen3/Eigen/Eigen>
#include <mutex>
#include <optional>
#include <set>
#include <unordered_map>

#include "rcs/Camera.h"
#include "sim/sim.h"

namespace rcs {
namespace sim {

enum CameraType {
  free = mjCAMERA_FREE,
  tracking = mjCAMERA_TRACKING,
  fixed = mjCAMERA_FIXED,
  default_free
};

struct SimCameraConfig : common::BaseCameraConfig {
  CameraType type;
  SimCameraConfig(const std::string& identifier, int frame_rate,
                  int resolution_width, int resolution_height,
                  CameraType type = fixed)
      : common::BaseCameraConfig(identifier, frame_rate, resolution_width,
                                 resolution_height),
        type(type) {}
};

// (H,W,3)
typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> ColorFrame;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> DepthFrame;

struct FrameSet {
  std::unordered_map<std::string, ColorFrame> color_frames;
  std::unordered_map<std::string, DepthFrame> depth_frames;
  mjtNum timestamp;
};

class SimCameraSet {
 public:
  SimCameraSet(std::shared_ptr<rcs::sim::Sim> sim,
               std::unordered_map<std::string, SimCameraConfig> cameras,
               bool render_on_demand = true);
  ~SimCameraSet();

  int buffer_size();
  void clear_buffer();

  std::optional<FrameSet> get_latest_frameset();
  std::optional<FrameSet> get_timestamp_frameset(float ts);

  void frame_callback(const std::string& id, mjrContext& ctx, mjvScene& scene,
                      mjvOption& opt);

  std::shared_ptr<Sim> get_sim() { return sim; }
  bool render_on_demand;

 private:
  std::shared_ptr<Sim> sim;
  std::unordered_map<std::string, SimCameraConfig> cameras_cfg;
  std::vector<FrameSet> buffer;
  std::unordered_map<std::string, mjvCamera> cameras;
  std::mutex buffer_lock;
  mjtNum last_ts = 0;
  void render_all();
};
}  // namespace sim
}  // namespace rcs
#endif  // RCS_CAMSIM_H
