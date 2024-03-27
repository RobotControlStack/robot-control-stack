#include "sim.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "common/Pose.h"
#include "glfw_adapter.h"
#include "mujoco/mjdata.h"
#include "mujoco/mujoco.h"
#include "platform_ui_adapter.h"
#include "rl/math/Rotation.h"
#include "rl/math/Transform.h"
#include "rl/math/Vector.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/Kinematic.h"
#include "rl/mdl/UrdfFactory.h"

Simulation::Simulation(std::shared_ptr<mjModel> fr3_mjmdl,
                       std::vector<std::shared_ptr<rl::mdl::Model>> fr3_rlmdls,
                       bool render, size_t n_threads)
    : fr3_mjmdl(fr3_mjmdl),
      fr3_rlmdls(fr3_rlmdls),
      targets(0),
      mujoco_data(n_threads),
      physics_threads(0),
      sync_point(n_threads, std::bind(&Simulation::syncfn, this)),
      semaphores(0, 0),
      exit_requested(false) {
  targets.reserve(n_threads);
  for (size_t i = 0; i < n_threads; ++i) {
    targets.emplace_back(std::make_shared<rcs::common::Pose>());
  }
  for (size_t i = 0; i < n_threads; ++i) {
    this->mujoco_data[i] =
        std::shared_ptr<mjData>(mj_makeData(this->fr3_mjmdl.get()));
  }
  for (size_t i = 0; i < n_threads; ++i) {
    physics_threads.push_back(std::jthread(
        std::bind(&Simulation::physics_loop, this, this->mujoco_data[i],
                  this->targets[i], this->fr3_rlmdls[i])));
  }
  if (render) {
    render_thread = std::jthread(std::bind(&Simulation::render_loop, this));
  }
  // wait for physics threads to signal that they are ready
  this->semaphores.second.acquire();
}

rl::math::Matrix Simulation::reset() {
  size_t state_size = mj_stateSize(fr3_mjmdl.get(), mjSTATE_PHYSICS);
  Eigen::Matrix<rl::math::Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      ret(std::size(physics_threads), state_size);  // row major like numpy
  for (size_t i = 0; i < std::size(physics_threads); ++i) {
    mj_resetDataKeyframe(fr3_mjmdl.get(), mujoco_data[i].get(), SIM_KF_HOME);
    mj_getState(fr3_mjmdl.get(), mujoco_data[i].get(), &ret(i, 0),
                mjSTATE_PHYSICS);
  }
  return ret;
}

void Simulation::physics_loop(
    std::shared_ptr<mjData> data,
    std::shared_ptr<rcs::common::Pose> target_transform,
    std::shared_ptr<rl::mdl::Model> rlmdl) {
  std::shared_ptr<rl::mdl::Kinematic> kin =
      std::dynamic_pointer_cast<rl::mdl::Kinematic>(rlmdl);
  rl::mdl::JacobianInverseKinematics ik(kin.get());
  ik.setDuration(std::chrono::milliseconds(100));
  rl::math::Vector q(7);
  rl::math::Transform last_pos, current_pos;
  this->sync_point.arrive_and_wait();
  this->semaphores.first.acquire();
  double eps = 0.00001;
  while (not exit_requested) {
    {  // This is one step
      // Solve IK
      bool converged = false;
      ik.addGoal(target_transform->affine_matrix(), 0);
      bool ik_success = ik.solve();
      if (ik_success) {
        // Set control in mujoco sim
        kin->forwardPosition();
        for (size_t i = 0; i < std::size(q); ++i) {
          data->ctrl[i] = kin->getPosition()[i];
          q[i] = data->qpos[i];
        }
        kin->setPosition(q);
        kin->forwardPosition();
        last_pos = kin->getOperationalPosition(0);
      }
      // Wait until asked position is reached
      while (not converged and ik_success) {
        mj_step(this->fr3_mjmdl.get(), data.get());
        // Get current TCP coordinates
        for (size_t i = 0; i < std::size(q); ++i) {
          q[i] = data->qpos[i];
        }
        kin->setPosition(q);
        kin->forwardPosition();
        current_pos = kin->getOperationalPosition(0);
        converged = (current_pos.matrix().array() - last_pos.matrix().array())
                        .abs()
                        .sum() < eps;
        last_pos = current_pos;
      }
    }
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
  mjv_makeScene(this->fr3_mjmdl.get(), &scn, kMaxGeom);
  mjv_defaultFreeCamera(this->fr3_mjmdl.get(), &cam);

  ui_adapter.RefreshMjrContext(this->fr3_mjmdl.get(), 1);
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
    mjv_updateScene(this->fr3_mjmdl.get(), this->mujoco_data[current_sim].get(),
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

rl::math::Matrix Simulation::step(std::vector<rcs::common::Pose> act) {
  // set IK targets
  for (size_t i = 0; i < std::size(this->targets); ++i) {
    *this->targets[i] = act[i];
  }
  // Do step
  this->semaphores.first.release(this->physics_threads.size());
  this->semaphores.second.acquire();
  // Process step and return state
  size_t state_size = mj_stateSize(fr3_mjmdl.get(), mjSTATE_PHYSICS);
  Eigen::Matrix<rl::math::Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      ret(std::size(physics_threads), state_size);  // row major like numpy
  for (size_t i = 0; i < std::size(physics_threads); ++i) {
    mj_getState(fr3_mjmdl.get(), mujoco_data[i].get(), &ret(i, 0),
                mjSTATE_PHYSICS);
  }
  return ret;
}

struct sim {
  Simulation* s;
  std::vector<std::shared_ptr<rl::mdl::Model>>* fr3_rlmdls;
  std::shared_ptr<mjModel> fr3_mjmdl;
};

sim* sim_init(size_t n_threads) {
  rl::mdl::UrdfFactory factory{};
  std::vector<std::shared_ptr<rl::mdl::Model>>* fr3_rlmdls =
      new std::vector<std::shared_ptr<rl::mdl::Model>>(0);
  fr3_rlmdls->reserve(n_threads);
  for (size_t i = 0; i < n_threads; ++i) {
    fr3_rlmdls->emplace_back(
        factory.create(MODEL_DIR "/urdf/fr3_from_panda.urdf"));
  }
  std::shared_ptr<mjModel> fr3_mjmdl = std::shared_ptr<mjModel>(
      mj_loadXML(MODEL_DIR "/mjcf/scene.xml", NULL, NULL, 0));
  Simulation* sim = new Simulation(fr3_mjmdl, *fr3_rlmdls, true, n_threads);
  struct sim* ret = new struct sim();
  ret->s = sim;
  ret->fr3_rlmdls = fr3_rlmdls;
  ret->fr3_mjmdl = fr3_mjmdl;
  return ret;
}
