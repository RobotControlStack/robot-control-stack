#include <mujoco/mujoco.h>

#include <boost/interprocess/creation_tags.hpp>
#include <iostream>

#include "sim.h"
#include "sim/gui.h"

namespace rcs {
namespace sim {

GuiServer::GuiServer(mjModel* m, mjData* d, const std::string& id)
    : m{m},
      d{d},
      shm{.state{.size = calculate_state_size(m)},
          .model{.size = calculate_mdl_size(m)},
          .manager{create_only, id.c_str(), calculate_shm_size(m, d)},
          .state_lock{create_only, (id + STATE_LOCK_POSTFIX).c_str()},
          .info_lock{create_only, (id + INFO_LOCK_POSTFIX).c_str()}},
      id{id} {
  this->shm.state.ptr =
      this->shm.manager.construct<mjtNum>(STATE)[this->shm.state.size]();
  this->shm.model.ptr =
      this->shm.manager.construct<char>(MODEL)[this->shm.model.size]();
  this->shm.info_byte = this->shm.manager.construct<bool>(INFO_BYTE)(0);
  mj_getState(m, d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
  mj_saveModel(m, NULL, this->shm.model.ptr, this->shm.model.size);
}

GuiServer::~GuiServer() {
  bool ret =
      boost::interprocess::shared_memory_object::remove(this->id.c_str());
  if (not ret) {
    std::cout << "Failed deleting the shared memory named " << this->id
              << std::endl;
  }
  ret = named_sharable_mutex::remove((id + STATE_LOCK_POSTFIX).c_str());
  if (not ret) {
    std::cout << "Failed deleting the mutex named "
              << (this->id + STATE_LOCK_POSTFIX) << std::endl;
  }
  ret = named_upgradable_mutex::remove((id + INFO_LOCK_POSTFIX).c_str());
  if (not ret) {
    std::cout << "Failed deleting the mutex named "
              << (this->id + INFO_LOCK_POSTFIX) << std::endl;
  }
};

void GuiServer::update_mjdata_callback() {
  this->shm.info_lock.lock_upgradable();
  if (*this->shm.info_byte) {
    this->shm.state_lock.lock();
    mj_getState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
    this->shm.state_lock.unlock();
    this->shm.info_lock.unlock_upgradable_and_lock();
    *this->shm.info_byte = false;
    this->shm.info_lock.unlock_and_lock_upgradable();
  }
  this->shm.info_lock.unlock_upgradable();
}

}  // namespace sim
}  // namespace rcs
