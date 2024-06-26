#ifndef RCS_SIM_H
#define RCS_SIM_H
#include <functional>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace rcs {
namespace sim {

class Renderer {
 public:
  Renderer(mjModel* m);
  ~Renderer();
  size_t register_context(size_t width, size_t height, bool offscreen);
  mjrContext& get_context(size_t id);
  mjvScene scene;

 private:
  mjModel* m;
  std::vector<std::pair<GLFWwindow*, mjrContext>> ctxs;
  mjvOption opt;
};

struct Config {
  bool async = false;
  bool realtime = false;
};

struct Callback {
  std::function<void(void)> cb;
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;
};

struct ConditionCallback {
  std::function<bool(void)> cb;
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;    // in seconds
  bool last_return_value;
};

struct RenderingCallback {
  std::function<void(mjrContext&, mjvScene&)> cb;
  size_t id;                     // rendering context id in renderer class
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;    // in seconds
};

class Sim {
 private:
  Config cfg;
  rcs::sim::Renderer renderer;
  std::vector<Callback> callbacks;
  std::vector<ConditionCallback> any_callbacks;
  std::vector<ConditionCallback> all_callbacks;
  std::vector<RenderingCallback> rendering_callbacks;
  void invoke_callbacks();
  bool invoke_condition_callbacks();
  void invoke_rendering_callbacks();

 public:
  // TODO: hide m & d, pass as parameter to callback (easier refactoring)
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
  void register_any_cb(std::function<bool(void)> cb,
                       mjtNum seconds_between_calls);
  void register_all_cb(std::function<bool(void)> cb,
                       mjtNum seconds_between_calls);
  void register_rendering_callback(
      std::function<void(mjrContext&, mjvScene&)> cb,
      mjtNum seconds_between_calls, size_t width, size_t height,
      bool offscreen);
};
}  // namespace sim
}  // namespace rcs
#endif
