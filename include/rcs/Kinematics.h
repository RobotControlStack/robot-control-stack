#ifndef RCS_IK_H
#define RCS_IK_H

#include <optional>

#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

class Kinematics {
 public:
  virtual ~Kinematics(){};
  virtual std::optional<VectorXd> inverse(
      const Pose& pose, const VectorXd& q0,
      const Pose& tcp_offset = Pose::Identity()) = 0;
  virtual Pose forward(const VectorXd& q0, const Pose& tcp_offset) = 0;
};

}  // namespace common
}  // namespace rcs

#endif  // RCS_IK_H
