#ifndef RCS_SIM_H
#define RCS_SIM_H
#include <functional>
#include <string>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace rcs {
namespace sim {

class Renderer {
 public:
  Renderer(mjModel* m);
  ~Renderer();
  void register_context(const std::string& id, size_t width, size_t height,
                        bool offscreen);
  mjrContext* get_context(const std::string& id);
  mjvScene scene;
  mjvOption opt;

 private:
  mjModel* m;
  std::unordered_map<std::string, std::pair<GLFWwindow*, mjrContext*>> ctxs;
};

struct Config {
  bool async = false;
  bool realtime = false;
  int max_convergence_steps = 5000;
};

struct Callback {
  const std::function<void(void)> cb;
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;
};

struct ConditionCallback {
  const std::function<bool(void)> cb;
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;    // in seconds
  bool last_return_value;
};

struct RenderingCallback {
  const std::function<void(const std::string&, mjrContext&, mjvScene&,
                           mjvOption&)>
      cb;
  const std::string id;          // rendering context id in renderer class
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;    // in seconds
};

class Sim {
 private:
  Config cfg;
  std::vector<Callback> callbacks;
  std::vector<ConditionCallback> any_callbacks;
  std::vector<ConditionCallback> all_callbacks;
  std::vector<RenderingCallback> rendering_callbacks;
  void invoke_callbacks();
  bool invoke_condition_callbacks();
  void invoke_rendering_callbacks();
  size_t convergence_steps = 0;
  bool converged = true;

 public:
  // TODO: hide m & d, pass as parameter to callback (easier refactoring)
  rcs::sim::Renderer renderer;
  mjModel* m;
  mjData* d;
  Sim(mjModel* m, mjData* d);
  bool set_config(const Config& cfg);
  Config get_config();
  bool is_converged();
  void step_until_convergence();
  void step(size_t k);
  void reset();
  /* NOTE: IMPORTANT, the callback is not necessarily called at exactly the
   * the requested interval. We invoke a callback if the elapsed simulation time
   * since the last call of the callback is greater than the requested time.
   * The exact interval at which they are called depends on the models timestep
   * option (cf.
   * https://mujoco.readthedocs.io/en/stable/programming/simulation.html#simulation-loop
   */
  void register_cb(std::function<void(void)> cb, mjtNum seconds_between_calls);
  void register_any_cb(std::function<bool(void)> cb,
                       mjtNum seconds_between_calls);
  void register_all_cb(std::function<bool(void)> cb,
                       mjtNum seconds_between_calls);
  void register_rendering_callback(
      std::function<void(const std::string& id, mjrContext&, mjvScene&,
                         mjvOption&)>
          cb,
      const std::string& id, mjtNum seconds_between_calls, size_t width,
      size_t height, bool offscreen);
};
}  // namespace sim
}  // namespace rcs
#endif
