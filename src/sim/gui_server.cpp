#include <mujoco/mujoco.h>

#include <iostream>

#include "sim.h"

namespace rcs {
namespace sim {

GuiServer::GuiServer(mjModel* m, mjData* d, const std::string& id)
    : m{m},
      d{d},
      shm{.state{.size = calculate_state_size(m)},
          .model{.size = calculate_mdl_size(m)},
          .manager{boost::interprocess::managed_shared_memory(
              boost::interprocess::create_only, id.c_str(),
              calculate_shm_size(m, d))}},
      id{id} {
  this->shm.state.ptr =
      this->shm.manager.construct<mjtNum>(STATE)[this->shm.state.size]();
  this->shm.model.ptr =
      this->shm.manager.construct<char>(MODEL)[this->shm.model.size]();
  this->shm.info_byte = this->shm.manager.construct<char>(INFO_BYTE)(0);
  mj_step(m, d);  // TODO: check if this is necessary
  mj_getState(m, d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
  mj_saveModel(m, NULL, this->shm.model.ptr, this->shm.model.size);
}

GuiServer::~GuiServer() {
  bool ret = boost::interprocess::shared_memory_object::remove(this->id.c_str());
  if (not ret) {
    std::cout << "Failed deleting the shared memory named " << this->id << std::endl;
  }
};

void GuiServer::update_mjdata_callback() {
  if (this->shm.info_byte and not(*this->shm.info_byte & UPDATE_SIM_STATE)) {
    mj_getState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
    *this->shm.info_byte &= ~UPDATE_SIM_STATE;
  }
}

}  // namespace sim
}  // namespace rcs
