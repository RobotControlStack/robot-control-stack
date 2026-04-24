#ifndef RCS_CAMERA_H
#define RCS_CAMERA_H
#include <string>

namespace rcs {
namespace common {

struct BaseCameraConfig {
  std::string identifier;
  int frame_rate;
  int resolution_width;
  int resolution_height;
  virtual ~BaseCameraConfig() {}
};

}  // namespace common
}  // namespace rcs

#endif  // RCS_CAMERA_H