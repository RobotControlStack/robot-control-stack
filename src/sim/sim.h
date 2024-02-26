#include <barrier>
#include <memory>
#include <string>
#include <thread>

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
  std::shared_ptr<mjModel> model;
  std::vector<std::shared_ptr<mjData>> mujoco_data;
  std::jthread render_thread;
  std::vector<std::jthread> physics_threads;
  std::barrier<std::function<void(void)>> sync_point;
  std::pair<std::counting_semaphore<SIM_MAX_THREADS>, std::binary_semaphore>
      semaphores;
  bool exit_requested;
  void syncfn() noexcept;
  void render_loop();
  void physics_loop(std::shared_ptr<mjData> data);

 public:
  Simulation(std::shared_ptr<mjModel> model, bool render, size_t n_threads);
  rl::math::Matrix reset();
  rl::math::Matrix step(rl::math::Matrix act);
  void close();
};
