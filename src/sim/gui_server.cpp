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
  *this->shm.info_byte &= SERVER_SHUTTING_DOWN;
  while (not(*this->shm.info_byte & CLIENT_OPEN)) {
  };
  this->shm.manager.destroy<char>(INFO_BYTE);
  this->shm.manager.destroy<mjtNum>(STATE);
  this->shm.manager.destroy<char>(MODEL);
  // NOTE: this is only deleted if no process has this shm mapped.
  // i.e. we don't even need to check if someone is using the memory.
  // What we need to do is destroy the the managed_shared_memory object. This
  // will unmap the shared memory object and free all resources. Then we need to
  // delete the shm from the system.
  boost::interprocess::shared_memory_object::remove(this->id.c_str());
};

void GuiServer::update_mjdata_callback() {
  if (this->shm.info_byte and not(*this->shm.info_byte & UPDATE_SIM_STATE)) {
    mj_getState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
    *this->shm.info_byte &= ~UPDATE_SIM_STATE;
  }
}

}  // namespace sim
}  // namespace rcs
