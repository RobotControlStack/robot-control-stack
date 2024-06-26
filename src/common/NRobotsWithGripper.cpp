#include "NRobotsWithGripper.h"

namespace rcs {
namespace common {

NRobotsWithGripper::NRobotsWithGripper(
    std::vector<std::shared_ptr<RobotWithGripper>> robots_with_gripper)
    : robots_with_gripper(robots_with_gripper) {}

NRobotsWithGripper::~NRobotsWithGripper() {}

// ROBOT FUNCTIONS
std::vector<std::unique_ptr<RConfig>> NRobotsWithGripper::get_parameters_r(
    const std::vector<size_t> &idxs) {
  std::function<std::unique_ptr<RConfig>(size_t)> f = [this, &idxs](size_t i) {
    return std::unique_ptr<RConfig>(
        this->robots_with_gripper[idxs[i]]->robot->get_parameters());
  };
  return NRobotsWithGripper::execute_parallel(f, idxs.size());
}

std::vector<std::unique_ptr<RState>> NRobotsWithGripper::get_state_r(
    const std::vector<size_t> &idxs) {
  std::function<std::unique_ptr<RState>(size_t)> f = [this, &idxs](size_t i) {
    return std::unique_ptr<RState>(
        this->robots_with_gripper[idxs[i]]->robot->get_state());
  };
  return NRobotsWithGripper::execute_parallel(f, idxs.size());
}

std::vector<Pose> NRobotsWithGripper::get_cartesian_position(
    const std::vector<size_t> &idxs) {
  std::function<Pose(size_t)> f = [this, &idxs](size_t i) {
    return this->robots_with_gripper[idxs[i]]->robot->get_cartesian_position();
  };
  return NRobotsWithGripper::execute_parallel(f, idxs.size());
}

void NRobotsWithGripper::set_joint_position(const std::vector<size_t> &idxs,
                                            const std::vector<Vector7d> &q) {
  assert(idxs.size() == q.size());
  std::function<void(size_t)> f = [this, &idxs, &q](size_t i) {
    this->robots_with_gripper[idxs[i]]->robot->set_joint_position(q[i]);
  };
  NRobotsWithGripper::execute_parallel(f, idxs.size());
}

std::vector<Vector7d> NRobotsWithGripper::get_joint_position(
    const std::vector<size_t> &idxs) {
  std::function<Vector7d(size_t)> f = [this, &idxs](size_t i) {
    return this->robots_with_gripper[idxs[i]]->robot->get_joint_position();
  };
  return NRobotsWithGripper::execute_parallel(f, idxs.size());
}

void NRobotsWithGripper::move_home(const std::vector<size_t> &idxs) {
  std::function<void(size_t)> f = [this, &idxs](size_t i) {
    return this->robots_with_gripper[idxs[i]]->robot->move_home();
  };
  NRobotsWithGripper::execute_parallel(f, idxs.size());
}

void NRobotsWithGripper::set_cartesian_position(const std::vector<size_t> &idxs,
                                                const std::vector<Pose> &pose) {
  assert(idxs.size() == pose.size());
  std::function<void(size_t)> f = [this, &idxs, &pose](size_t i) {
    return this->robots_with_gripper[idxs[i]]->robot->set_cartesian_position(
        pose[i]);
  };
  NRobotsWithGripper::execute_parallel(f, idxs.size());
}

// GRIPPER FUNCTIONS
std::vector<std::optional<std::unique_ptr<GConfig>>>
NRobotsWithGripper::get_parameters_g(const std::vector<size_t> &idxs) {
  std::function<std::optional<std::unique_ptr<GConfig>>(size_t)> f =
      [this, &idxs](size_t i) -> std::optional<std::unique_ptr<GConfig>> {
    if (this->robots_with_gripper[idxs[i]]->gripper.has_value()) {
      return std::unique_ptr<GConfig>(this->robots_with_gripper[idxs[i]]
                                          ->gripper.value()
                                          ->get_parameters());
    }
    return std::nullopt;
  };
  return NRobotsWithGripper::execute_parallel(f, idxs.size());
}

std::vector<std::optional<std::unique_ptr<GState>>>
NRobotsWithGripper::get_state_g(const std::vector<size_t> &idxs) {
  std::function<std::optional<std::unique_ptr<GState>>(size_t)> f =
      [this, &idxs](size_t i) -> std::optional<std::unique_ptr<GState>> {
    if (this->robots_with_gripper[idxs[i]]->gripper.has_value()) {
      return std::unique_ptr<GState>(
          this->robots_with_gripper[idxs[i]]->gripper.value()->get_state());
    }
    return std::nullopt;
  };
  return NRobotsWithGripper::execute_parallel(f, idxs.size());
}

std::vector<std::optional<bool>> NRobotsWithGripper::grasp(
    const std::vector<size_t> &idxs) {
  std::function<std::optional<bool>(size_t)> f =
      [this, &idxs](size_t i) -> std::optional<bool> {
    if (this->robots_with_gripper[idxs[i]]->gripper.has_value()) {
      return this->robots_with_gripper[idxs[i]]->gripper.value()->grasp();
    }
    return std::nullopt;
  };
  return NRobotsWithGripper::execute_parallel(f, idxs.size());
}

void NRobotsWithGripper::release(const std::vector<size_t> &idxs) {
  std::function<void(size_t)> f = [this, &idxs](size_t i) {
    if (this->robots_with_gripper[idxs[i]]->gripper.has_value()) {
      this->robots_with_gripper[idxs[i]]->gripper.value()->release();
    }
  };
  NRobotsWithGripper::execute_parallel(f, idxs.size());
}

void NRobotsWithGripper::shut(const std::vector<size_t> &idxs) {
  std::function<void(size_t)> f = [this, &idxs](size_t i) {
    if (this->robots_with_gripper[idxs[i]]->gripper.has_value()) {
      this->robots_with_gripper[idxs[i]]->gripper.value()->shut();
    }
  };
  NRobotsWithGripper::execute_parallel(f, idxs.size());
}

}  // namespace common
}  // namespace rcs