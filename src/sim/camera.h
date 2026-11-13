#ifndef RCS_CAMSIM_H
#define RCS_CAMSIM_H
#include <mujoco/mujoco.h>

#include "rcs/Camera.h"

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
