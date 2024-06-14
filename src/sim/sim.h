#ifndef RCS_SIM_H
#define RCS_SIM_H
#include <functional>

#include "mujoco/mujoco.h"

namespace rcs {
namespace sim {

struct Config {
  bool async;
  bool realtime;
};

struct Callback {
  std::function<void(void)> cb;
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;
};

struct ConvergenceCallback {
  std::function<bool(void)> cb;
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;    // in seconds
};

class Sim {
 private:
  Config cfg;
  size_t step_count;
  std::vector<Callback> callbacks;
  std::vector<ConvergenceCallback> convergence_callbacks;
  void invoke_callbacks();
  bool invoke_convergence_callbacks();

 public:
  mjModel* m;
  mjData* d;
  Sim(mjModel* m, mjData* d);
  bool set_config(const Config& cfg);
  Config get_config();
  void step_until_convergence();
  void step(size_t k);
  /* NOTE: IMPORTANT, the callback is not necessarily called at exactly the
   * the requested interval. We invoke a callback if the elapsed simulation time
   * since the last call of the callback is greater than the requested time.
   * The exact interval at which they are called depends on the models timestep
   * option (cf.
   * https://mujoco.readthedocs.io/en/stable/programming/simulation.html#simulation-loop
   */
  void register_cb(std::function<void(void)> cb, mjtNum seconds_between_calls);
  void register_convergence_cb(std::function<bool(void)> cb,
                               mjtNum seconds_between_calls);
};
}  // namespace sim
}  // namespace rcs
#endif
