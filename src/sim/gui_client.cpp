#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>

#include <functional>
#include <iostream>

#include "glfw_adapter.h"
#include "gui.h"
#include "platform_ui_adapter.h"

namespace rcs {
namespace sim {
using namespace boost::interprocess;

void event_callback(mjuiState*);
class GuiClient {
 public:
  GuiClient(const std::string& id);
  void render_loop();
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjModel* m;
  mjData* d;
  mujoco::PlatformUIAdapter* platform_ui;

 private:
  const std::string& id;
  struct shm shm;
};

GuiClient::GuiClient(const std::string& id)
    : shm{.manager{open_only, id.c_str()},
          .state_lock{open_only, (id + STATE_LOCK_POSTFIX).c_str()},
          .info_lock{open_only, (id + INFO_LOCK_POSTFIX).c_str()}},
      id{id} {
  // setup shared memory
  std::tie(this->shm.info_byte, std::ignore) =
      this->shm.manager.find<bool>(INFO_BYTE);
  std::tie(this->shm.state.ptr, this->shm.state.size) =
      this->shm.manager.find<mjtNum>(STATE);
  std::tie(this->shm.model.ptr, this->shm.model.size) =
      this->shm.manager.find<char>(MODEL);
  // initialize model and data from shared memory
  mjVFS* vfs = (mjVFS*)mju_malloc(sizeof(mjVFS));
  mj_defaultVFS(vfs);
  int failed = mj_addBufferVFS(vfs, "model.mjb", this->shm.model.ptr,
                               this->shm.model.size);
  if (failed != 0) {
    mju_free(vfs);
    throw std::runtime_error("Could not add file to VFS");
  }
  this->m = mj_loadModel("model.mjb", vfs);
  mju_free(vfs);
  this->d = mj_makeData(m);
  this->shm.state_lock.lock_sharable();
  mj_setState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
  this->shm.state_lock.unlock_sharable();
  mj_step(this->m, this->d);
  // start UI window
  this->platform_ui = new mujoco::GlfwAdapter();
  this->render_loop();
}

void GuiClient::render_loop() {
  mjv_defaultFreeCamera(this->m, &this->cam);
  mjv_defaultOption(&this->opt);
  mjv_defaultScene(&this->scn);
  mjv_makeScene(this->m, &this->scn, mjFONTSCALE_100);
  mjrRect viewport = {0, 0, 0, 0};
  this->platform_ui->RefreshMjrContext(this->m, mjFONTSCALE_100);
  this->platform_ui->SetVSync(true);
  this->platform_ui->state().userdata = this;
  this->platform_ui->state().nrect = 1;
  this->platform_ui->SetEventCallback(event_callback);
  auto last_cam_distance = this->cam.distance;
  while (!this->platform_ui->ShouldCloseWindow()) {
    this->platform_ui->PollEvents();
    if (this->cam.distance != last_cam_distance) {
      std::cout << "cam.distance after PollEvents: " << this->cam.distance
                << std::endl;
    }
    this->shm.state_lock.lock_sharable();
    mj_setState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
    this->shm.state_lock.unlock_sharable();
    mj_step(this->m, this->d);
    if (this->cam.distance != last_cam_distance) {
      std::cout << "cam.distance after step: " << this->cam.distance
                << std::endl;
    }
    mjv_updateScene(this->m, this->d, &this->opt, NULL, &this->cam, mjCAT_ALL,
                    &this->scn);
    if (this->cam.distance != last_cam_distance) {
      std::cout << "cam.distance after update_scene: " << this->cam.distance
                << std::endl;
    }
    last_cam_distance = this->cam.distance;
    std::tie(viewport.width, viewport.height) =
        this->platform_ui->GetFramebufferSize();
    mjr_render(viewport, &this->scn, &this->platform_ui->mjr_context());
    this->shm.info_lock.lock();
    *this->shm.info_byte = true;
    this->shm.info_lock.unlock();
    this->platform_ui->SwapBuffers();
  }
}
void open_gui_window(std::string& id) {
  rcs::sim::GuiClient gui = rcs::sim::GuiClient(id);
  gui.render_loop();
}

void event_callback(mjuiState* state) {
  GuiClient* client = static_cast<GuiClient*>(state->userdata);
  mjrRect viewport = {0, 0, 0, 0};
  std::tie(viewport.width, viewport.height) =
      client->platform_ui->GetFramebufferSize();
  auto reldy = -0.02 * state->sy;
  switch (state->type) {
    case mjEVENT_SCROLL:
      std::cout << "cam.distance: " << client->cam.distance << std::endl;
      std::cout << "m->stat.extent: " << client->m->stat.extent << std::endl;
      std::cout << "reldy: " << reldy << std::endl;
      std::cout << mju_log(1 +
                           client->cam.distance / client->m->stat.extent / 3) *
                       reldy * 9 * client->m->stat.extent
                << std::endl;
      mjv_moveCamera(client->m, mjMOUSE_ZOOM, 0, -0.02 * state->sy,
                     &client->scn, &client->cam);
      std::cout << "cam.distance after cam move: " << client->cam.distance
                << std::endl;
    case mjEVENT_MOVE:
      // determine action based on mouse button
      mjtMouse action;
      if (state->right) {
        action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
      } else if (state->left) {
        action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
      } else {
        action = mjMOUSE_ZOOM;
      }
      mjv_moveCamera(client->m, action, state->dx / viewport.height,
                     -state->dy / viewport.height, &client->scn, &client->cam);
  }
}
}  // namespace sim
}  // namespace rcs
