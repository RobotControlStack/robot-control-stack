#include "rcs/LinearInterpolator.h"

#include <algorithm>
#include <stdexcept>

namespace rcs {
namespace common {

namespace {
double check_duration(double duration) {
  if (!(duration > 0.0)) {
    throw std::invalid_argument("interpolation duration must be positive");
  }
  return duration;
}

// Fraction of the segment that is done, in [0, 1].
double progress(double elapsed, double duration) {
  return std::clamp(elapsed / duration, 0.0, 1.0);
}

// Rotation from `from` to `to` as a rotation vector in the base frame, along
// the shortest path.
Eigen::Vector3d rotation_between(const Eigen::Quaterniond& from,
                                 const Eigen::Quaterniond& to) {
  Eigen::Quaterniond delta = to * from.inverse();
  if (delta.w() < 0.0) {
    delta.coeffs() *= -1.0;  // same rotation, shorter way round
  }
  Eigen::AngleAxisd angle_axis(delta);
  return angle_axis.angle() * angle_axis.axis();
}
}  // namespace

LinearJointInterpolator::LinearJointInterpolator(double duration)
    : default_duration(check_duration(duration)) {}

void LinearJointInterpolator::reset(const VectorXd& q) {
  this->q = q;
  this->start = q;
  this->goal = q;
  this->duration = this->default_duration;
  this->elapsed = this->duration;  // nothing left to do: hold q
}

void LinearJointInterpolator::set_goal(const VectorXd& q_goal,
                                       std::optional<double> duration) {
  if (this->q.size() == 0) {
    throw std::logic_error("call reset() before set_goal()");
  }
  if (q_goal.size() != this->q.size()) {
    throw std::invalid_argument("goal has the wrong number of joints");
  }
  this->start = this->q;
  this->goal = q_goal;
  this->duration = check_duration(duration.value_or(this->default_duration));
  this->elapsed = 0.0;
}

JointSetpoint LinearJointInterpolator::step(double dt) {
  if (this->q.size() == 0) {
    throw std::logic_error("call reset() before step()");
  }
  this->elapsed += dt;
  double t = progress(this->elapsed, this->duration);
  this->q = this->start + t * (this->goal - this->start);
  VectorXd dq = VectorXd::Zero(this->q.size());
  if (t < 1.0) {
    dq = (this->goal - this->start) / this->duration;
  }
  return {this->q, dq};
}

LinearCartesianInterpolator::LinearCartesianInterpolator(double duration)
    : default_duration(check_duration(duration)) {}

void LinearCartesianInterpolator::reset(const Pose& pose) {
  this->pose = pose;
  this->start = pose;
  this->goal = pose;
  this->duration = this->default_duration;
  this->elapsed = this->duration;
  this->initialized = true;
}

void LinearCartesianInterpolator::set_goal(const Pose& pose_goal,
                                           std::optional<double> duration) {
  if (!this->initialized) {
    throw std::logic_error("call reset() before set_goal()");
  }
  this->start = this->pose;
  this->goal = pose_goal;
  this->duration = check_duration(duration.value_or(this->default_duration));
  this->elapsed = 0.0;
}

CartesianSetpoint LinearCartesianInterpolator::step(double dt) {
  if (!this->initialized) {
    throw std::logic_error("call reset() before step()");
  }
  this->elapsed += dt;
  double t = progress(this->elapsed, this->duration);
  // translation on a straight line, orientation by slerp (shortest path)
  this->pose = this->start.interpolate(this->goal, t);
  Vector6d twist = Vector6d::Zero();
  if (t < 1.0) {
    twist.head<3>() =
        (this->goal.translation() - this->start.translation()) / this->duration;
    twist.tail<3>() =
        rotation_between(this->start.quaternion(), this->goal.quaternion()) /
        this->duration;
  }
  return {this->pose, twist};
}

}  // namespace common
}  // namespace rcs
