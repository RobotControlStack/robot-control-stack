#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <memory>

#include "sim/sim.h"

class Renderer {
 public:
  Renderer(std::shared_ptr<rcs::sim::Sim> sim);
  ~Renderer();
  size_t register_context(size_t width, size_t height, bool offscreen);
  mjrContext& get_context(size_t id);
  mjvScene scene;

 private:
  std::shared_ptr<rcs::sim::Sim> sim;
  std::vector<std::pair<GLFWwindow*, mjrContext>> ctxs;
  mjvOption opt;
};
