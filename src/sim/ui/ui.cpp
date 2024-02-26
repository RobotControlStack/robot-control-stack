#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <unistd.h>

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <tuple>

#include "../sim.h"
#include "glfw_adapter.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjui.h"
#include "mujoco/mjvisualize.h"
#include "platform_ui_adapter.h"

using namespace mujoco;

static constexpr int kMaxGeom = 20000;
static mjvCamera cam;
static mjvOption opt;
static mjvScene scn;
static mjvPerturb pert;
static size_t current_sim = 0;

static struct render_args args = {};

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // left arrow: previous thread
  if (act == GLFW_PRESS && key == GLFW_KEY_LEFT) {
    if (current_sim > 0) --current_sim;
  }
  // right arrow: next thread
  if (act == GLFW_PRESS && key == GLFW_KEY_RIGHT) {
    if (current_sim < args.n_threads - 1) ++current_sim;
  }
}

int ComputeFontScale(const PlatformUIAdapter& platform_ui) {
  // compute framebuffer-to-window ratio
  int buf_width, buf_height, win_width, win_height = 0;
  std::tie(buf_width, buf_height) = platform_ui.GetFramebufferSize();
  std::tie(win_width, win_height) = platform_ui.GetWindowSize();
  double b2w = static_cast<double>(buf_width) / win_width;

  // compute PPI
  double PPI = b2w * platform_ui.GetDisplayPixelsPerInch();

  // estimate font scaling, guard against unrealistic PPI
  int fs;
  if (buf_width > win_width) {
    fs = mju_round(b2w * 100);
  } else if (PPI > 50 && PPI < 350) {
    fs = mju_round(PPI);
  } else {
    fs = 150;
  }
  fs = mju_round(fs * 0.02) * 50;
  fs = mjMIN(300, mjMAX(100, fs));
  return fs;
}

int render_thread(void* thrd_data) {
  args = *(struct render_args*)thrd_data;
  free(thrd_data);
  GlfwAdapter ui_adapter = GlfwAdapter();
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjv_makeScene(args.model, &scn, kMaxGeom);
  mjv_defaultFreeCamera(args.model, &cam);
  int fontscale = ComputeFontScale(ui_adapter);
  ui_adapter.RefreshMjrContext(args.model, fontscale);
  mjuiState uistate = ui_adapter.state();
  std::memset(&uistate, 0, sizeof(mjuiState));
  uistate.nrect = 1;
  std::tie(uistate.rect[0].width, uistate.rect[0].height) =
      ui_adapter.GetFramebufferSize();
  if (!ui_adapter.IsGPUAccelerated()) {
    scn.flags[mjRND_SHADOW] = 0;
    scn.flags[mjRND_REFLECTION] = 0;
  }
  while (!ui_adapter.ShouldCloseWindow()) {
    std::tie(uistate.rect[0].width, uistate.rect[0].height) =
        ui_adapter.GetFramebufferSize();
    mjv_updateScene(args.model, args.data[current_sim], &opt, NULL, &cam,
                    mjCAT_ALL, &scn);
    mjr_render(uistate.rect[0], &scn, &ui_adapter.mjr_context());
    ui_adapter.SwapBuffers();
    ui_adapter.PollEvents();
  }
  return EXIT_SUCCESS;
}
