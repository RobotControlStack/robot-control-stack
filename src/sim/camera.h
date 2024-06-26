#ifndef RCS_CAMSIM_H
#define RCS_CAMSIM_H
#include <common/Pose.h>
#include <common/Robot.h>
#include <mujoco/mujoco.h>

#include <mutex>

#include "sim/sim.h"

namespace rcs {
namespace sim {

struct SimCameraConfig {
  std::map<std::string, std::string> camera2id;
  int frame_rate;
  int resolution_width;
  int resolution_height;
};

// H*W x 3
typedef Eigen::Matrix<char, -1, 3> ColorFrame;

struct FrameSet {
  // TODO: think about who manges the memory for the frames
  // probably simcam but where is it saved?
  std::map<std::string, ColorFrame> color_frames;
  mjtNum timestamp;
};

class SimCameraSet {
 public:
  SimCameraSet(std::shared_ptr<rcs::sim::Sim> sim, const SimCameraConfig& cfg);
  ~SimCameraSet();

  int buffer_size();
  void clear_buffer();

  FrameSet get_latest_frameset();
  FrameSet get_timestamp_frameset(float ts);
  /** TODO: method that returns all frames within a timestamp range */

  void frame_callback(mjrContext& ctx, mjvScene& scene);

 private:
  const SimCameraConfig cfg;
  std::shared_ptr<Sim> sim;
  std::vector<FrameSet> buffer;
  std::mutex buffer_lock;

  ColorFrame poll_frame(std::string camera_id, mjrContext& ctx,
                        mjvScene& scene);
};
}  // namespace sim
}  // namespace rcs
#endif  // RCS_CAMSIM_H