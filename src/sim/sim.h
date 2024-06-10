#include <condition_variable>
#include <functional>
#include <optional>
#include <thread>

#include "mujoco/mujoco.h"

namespace rcs {
namespace sim {

struct Config {
  bool async;
  bool realtime;
};
class Sim {
 private:
  Config cfg;
  size_t step_count;
  std::vector<std::function<void(void)>> callbacks;
  std::vector<std::function<bool(void)>> convergence_callbacks;

 public:
  mjModel* m;
  mjData* d;
  Sim(mjModel* m, mjData* d);
  bool set_config(const Config& cfg);
  Config get_config();
  void step_until_convergence();
  void step(size_t k);
  void register_cb(std::function<void(void)> cb);
  void register_convergence_cb(std::function<bool(void)> cb);
};
}  // namespace sim
}  // namespace rcs
