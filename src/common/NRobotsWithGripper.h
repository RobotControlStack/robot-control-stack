#ifndef RCS_NROBOTSWITHGRIPPER_H
#define RCS_NROBOTSWITHGRIPPER_H

#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "Pose.h"
#include "Robot.h"
#include "utils.h"

namespace rcs {
namespace common {

class NRobotsWithGripper {
 public:
  std::vector<std::shared_ptr<RobotWithGripper>> robots_with_gripper;

  NRobotsWithGripper(
      std::vector<std::shared_ptr<RobotWithGripper>> robots_with_gripper);
  // TODO: create constructor that takes ips and returns this object

  ~NRobotsWithGripper();

  template <typename T>
  static std::vector<T> execute_parallel(const std::function<T(size_t)> &func,
                                         size_t n) {
    std::vector<T> results;

    if (n == 1) {
      // no thread usage if there is only one robot
      results.push_back(func(0));
      return results;
    }

    // create new thread for each robot
    // std::vector<std::thread> threads;
    std::vector<std::future<T>> threads;
    for (size_t i = 0; i < n; i++) {
      threads.push_back(std::async(std::launch::async, func, i));
    }
    // collect results, no need to wait as get does this for us
    for (size_t i = 0; i < n; i++) {
      results.push_back(threads[i].get());
    }
    return results;
  }

  // special case for void functions
  static void execute_parallel(const std::function<void(size_t)> &func,
                               size_t n) {
    if (n == 1) {
      // no thread usage if there is only one robot
      func(0);
    }

    // create new thread for each robot
    std::vector<std::thread> threads;
    for (size_t i = 0; i < n; i++) {
      threads.push_back(std::thread(func, i));
    }
    // wait for all threads to finish
    for (size_t i = 0; i < n; i++) {
      threads[i].join();
    }
  }

  // ROBOT FUNCTIONS
  std::vector<std::unique_ptr<RConfig>> get_parameters_r(
      const std::vector<size_t> &idxs);

  std::vector<std::unique_ptr<RState>> get_state_r(
      const std::vector<size_t> &idxs);

  std::vector<Pose> get_cartesian_position(const std::vector<size_t> &idxs);

  void set_joint_position(const std::vector<size_t> &idxs,
                          const std::vector<Vector7d> &q);

  std::vector<Vector7d> get_joint_position(const std::vector<size_t> &idxs);
  
  std::vector<Vector7d> get_joint_velocity(const std::vector<size_t> &idxs);

  void move_home(const std::vector<size_t> &idxs);

  void set_cartesian_position(const std::vector<size_t> &idxs,
                              const std::vector<Pose> &pose);

  // GRIPPER FUNCTIONS
  std::vector<std::optional<std::unique_ptr<GConfig>>> get_parameters_g(
      const std::vector<size_t> &idxs);

  std::vector<std::optional<std::unique_ptr<GState>>> get_state_g(
      const std::vector<size_t> &idxs);

  void grasp(const std::vector<size_t> &idxs);

  void open(const std::vector<size_t> &idxs);

  void shut(const std::vector<size_t> &idxs);

  // missing functions: set_normalized_width, is_grasped, reset (also from
  // robot)
};

}  // namespace common
}  // namespace rcs

#endif  // RCS_NROBOTSWITHGRIPPER_H