#include "sim.h"

#include <iostream>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "glfw_adapter.h"
#include "mujoco/mjdata.h"
#include "mujoco/mujoco.h"
#include "rl/math/Rotation.h"
#include "rl/math/Transform.h"
#include "rl/math/Vector.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/Kinematic.h"
#include "rl/mdl/UrdfFactory.h"

Simulation::Simulation(std::shared_ptr<mjModel> model, bool render,
                       size_t n_threads)
    : model(model),
      mujoco_data(n_threads),
      physics_threads(0),
      sync_point(n_threads, std::bind(&Simulation::syncfn, this)),
      semaphores(0, 0),
      exit_requested(false) {
  for (size_t i = 0; i < n_threads; ++i) {
    this->mujoco_data[i] =
        std::shared_ptr<mjData>(mj_makeData(this->model.get()));
  }
  for (std::shared_ptr<mjData> data : this->mujoco_data) {
    physics_threads.push_back(
        std::jthread(std::bind(&Simulation::physics_loop, this, data)));
  }
  if (render) {
    render_thread = std::jthread(std::bind(&Simulation::render_loop, this));
  }
  // wait for physics threads to signal that they are ready
  this->semaphores.second.acquire();
}

rl::math::Matrix Simulation::reset() {
  size_t state_size = mj_stateSize(model.get(), mjSTATE_PHYSICS);
  Eigen::Matrix<rl::math::Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      ret(std::size(physics_threads), state_size);  // row major like numpy
  for (size_t i = 0; i < std::size(physics_threads); ++i) {
    mj_resetDataKeyframe(model.get(), mujoco_data[i].get(), SIM_KF_HOME);
    mj_getState(model.get(), mujoco_data[i].get(), &ret(i, 0), mjSTATE_PHYSICS);
  }
  return ret;
}

void Simulation::physics_loop(std::shared_ptr<mjData> data) {
  this->sync_point.arrive_and_wait();
  this->semaphores.first.acquire();
  double sim_time = 0;
  while (not exit_requested) {
    while (data->time - sim_time < 1) {  // simulate 1 second
      mj_step(this->model.get(), data.get());
    }
    sim_time = data->time;
    sync_point.arrive_and_wait();
    this->semaphores.first.acquire();
  }
  sync_point.arrive_and_wait();
}

void Simulation::render_loop() {
  constexpr int kMaxGeom = 20000;
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjvPerturb pert;
  size_t current_sim = 0;

  mujoco::GlfwAdapter ui_adapter = mujoco::GlfwAdapter();

  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjv_makeScene(this->model.get(), &scn, kMaxGeom);
  mjv_defaultFreeCamera(this->model.get(), &cam);

  ui_adapter.RefreshMjrContext(this->model.get(), 1);
  mjuiState uistate = ui_adapter.state();
  std::memset(&uistate, 0, sizeof(mjuiState));
  uistate.nrect = 1;
  std::tie(uistate.rect[0].width, uistate.rect[0].height) =
      ui_adapter.GetFramebufferSize();
  if (!ui_adapter.IsGPUAccelerated()) {
    scn.flags[mjRND_SHADOW] = 0;
    scn.flags[mjRND_REFLECTION] = 0;
  }
  while (!(ui_adapter.ShouldCloseWindow() or exit_requested)) {
    std::tie(uistate.rect[0].width, uistate.rect[0].height) =
        ui_adapter.GetFramebufferSize();
    mjv_updateScene(this->model.get(), this->mujoco_data[current_sim].get(),
                    &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(uistate.rect[0], &scn, &ui_adapter.mjr_context());
    ui_adapter.SwapBuffers();
    ui_adapter.PollEvents();
  }
}

void Simulation::close() {
  exit_requested = true;
  this->semaphores.first.release(std::size(this->physics_threads));
  this->semaphores.second.acquire();
}

void Simulation::syncfn() noexcept { this->semaphores.second.release(); }

rl::math::Matrix Simulation::step(rl::math::Matrix act) {
  // Do step
  this->semaphores.first.release(this->physics_threads.size());
  this->semaphores.second.acquire();
  // Process step and return state
  size_t state_size = mj_stateSize(model.get(), mjSTATE_PHYSICS);
  Eigen::Matrix<rl::math::Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      ret(std::size(physics_threads), state_size);  // row major like numpy
  for (size_t i = 0; i < std::size(physics_threads); ++i) {
    mj_getState(model.get(), mujoco_data[i].get(), &ret(i, 0), mjSTATE_PHYSICS);
  }
  return ret;
}
