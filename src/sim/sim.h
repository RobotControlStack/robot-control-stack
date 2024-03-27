#include <barrier>
#include <memory>
#include <string>
#include <thread>

#include "common/Pose.h"
#include "mujoco/mujoco.h"
#include "plugin.h"
#include "rl/math/Transform.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/Model.h"
#define SIM_MAX_THREADS 128

// Keyframes & their IDs.
enum { SIM_KF_HOME = 0 };

class Simulation {
 private:
  std::shared_ptr<mjModel> fr3_mjmdl;
  std::vector<std::shared_ptr<rl::mdl::Model>> fr3_rlmdls;
  std::vector<std::shared_ptr<mjData>> mujoco_data;
  std::vector<std::shared_ptr<rcs::common::Pose>> targets;
  std::jthread render_thread;
  std::vector<std::jthread> physics_threads;
  std::barrier<std::function<void(void)>> sync_point;
  std::pair<std::counting_semaphore<SIM_MAX_THREADS>, std::binary_semaphore>
      semaphores;
  bool exit_requested;
  void syncfn() noexcept;
  void render_loop();
  void physics_loop(std::shared_ptr<mjData> data,
                    std::shared_ptr<rcs::common::Pose> target_pose,
                    std::shared_ptr<rl::mdl::Model> rlmdl);

 public:
  Simulation(std::shared_ptr<mjModel> fr3_mjmdl,
             std::vector<std::shared_ptr<rl::mdl::Model>> fr3_rlmdls,
             bool render, size_t n_threads);
  rl::math::Matrix reset();
  rl::math::Matrix step(std::vector<rcs::common::Pose> act);
  void close();
};
