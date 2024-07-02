#ifndef RCS_MOTION_GENERATOR_H
#define RCS_MOTION_GENERATOR_H

#include <common/Robot.h>
#include <common/utils.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <Eigen/Core>
#include <array>

namespace rcs {
namespace hw {

/**
 * @file examples_common.h
 * Contains common types and functions for the examples.
 */

/**
 * Sets a default collision behavior, joint impedance and Cartesian impedance.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot& robot);

/**
 * An example showing how to generate a joint pose motion to a goal position.
 * Adapted from: Wisama Khalil and Etienne Dombre. 2002. Modeling,
 * Identification and Control of Robots (Kogan Page Science Paper edition).
 *
 */
class MotionGenerator {
 public:
  /**
   * Creates a new MotionGenerator instance for a target q.
   *
   * @param[in] speed_factor General speed factor in range [0, 1].
   * @param[in] q_goal Target joint positions.
   */
  MotionGenerator(double speed_factor, const common::Vector7d q_goal);

  /**
   * Sends joint position calculations
   *
   * @param[in] robot_state Current state of the robot.
   * @param[in] period Duration of execution.
   *
   * @return Joint positions for use inside a control loop.
   */
  franka::JointPositions operator()(const franka::RobotState& robot_state,
                                    franka::Duration period);

 private:
  bool calculateDesiredValues(double t, common::Vector7d* delta_q_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  const common::Vector7d q_goal_;

  common::Vector7d q_start_;
  common::Vector7d delta_q_;

  common::Vector7d dq_max_sync_;
  common::Vector7d t_1_sync_;
  common::Vector7d t_2_sync_;
  common::Vector7d t_f_sync_;
  common::Vector7d q_1_;

  double time_ = 0.0;

  common::Vector7d dq_max_ =
      (common::Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  common::Vector7d ddq_max_start_ =
      (common::Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  common::Vector7d ddq_max_goal_ =
      (common::Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
};
}  // namespace hw
}  // namespace rcs
#endif  // RCS_MOTION_GENERATOR_H