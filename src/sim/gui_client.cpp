#include <iostream>

#include "glfw_adapter.h"
#include "gui.h"
#include "platform_ui_adapter.h"

namespace rcs {
namespace sim {

class GuiClient {
 public:
  GuiClient(const std::string& id);
  void render_loop();

 private:
  const std::string& id;
  struct shm shm;
  mjModel* m;
  mjData* d;
  mujoco::PlatformUIAdapter* platform_ui;
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjvPerturb pert;
};

GuiClient::GuiClient(const std::string& id)
    : shm{.manager{boost::interprocess::managed_shared_memory(
          boost::interprocess::open_only, id.c_str())}},
      id{id} {
  std::tie(this->shm.info_byte, std::ignore) =
      this->shm.manager.find<char>(INFO_BYTE);
  std::tie(this->shm.state.ptr, this->shm.state.size) =
      this->shm.manager.find<mjtNum>(STATE);
  std::tie(this->shm.model.ptr, this->shm.model.size) =
      this->shm.manager.find<char>(MODEL);

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
  mj_setState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
  mj_step(this->m, this->d);
  this->platform_ui = new mujoco::GlfwAdapter();
  this->render_loop();
}

void GuiClient::render_loop() {
  mjv_defaultCamera(&this->cam);
  mjv_defaultPerturb(&this->pert);
  mjv_defaultOption(&this->opt);
  mjv_defaultScene(&this->scn);
  this->platform_ui->RefreshMjrContext(this->m, mjFONTSCALE_100);
  mjv_makeScene(this->m, &this->scn, mjFONTSCALE_100);
  this->platform_ui->SetVSync(true);
  while (!this->platform_ui->ShouldCloseWindow()) {
    this->platform_ui->PollEvents();
    mjv_updateScene(this->m, this->d, &this->opt, NULL, &this->cam, mjCAT_ALL,
                    &this->scn);
    mjrRect viewport = {0, 0, 0, 0};
    auto dim = this->platform_ui->GetFramebufferSize();
    viewport.width = dim.first;
    viewport.height = dim.second;
    mjr_render(viewport, &this->scn, &this->platform_ui->mjr_context());
    *this->shm.info_byte &= UPDATE_SIM_STATE;
    mj_setState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
    mj_step(this->m, this->d);
    this->platform_ui->SwapBuffers();
  }
}
void open_gui_window(std::string& id) {
  rcs::sim::GuiClient gui = rcs::sim::GuiClient(id);
  gui.render_loop();
}
}  // namespace sim
}  // namespace rcs

int main(int argc, char** argv) {
  if (argc != 2) {
    throw std::runtime_error("Usage: gui <id>");
  }
  std::string id{argv[1]};
  rcs::sim::open_gui_window(id);
}
