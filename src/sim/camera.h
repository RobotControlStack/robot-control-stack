#ifndef RCS_CAMSIM_H
#define RCS_CAMSIM_H
#include <mujoco/mujoco.h>

#include "rcs/Camera.h"

// Simulation cameras are rendered on the Python side (rcs.camera.sim), only
// the configuration types live in C++.
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
};

}  // namespace sim
}  // namespace rcs
#endif  // RCS_CAMSIM_H
