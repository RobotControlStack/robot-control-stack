#include <functional>
#include <iostream>
#include <string>

#include "gui.h"

namespace rcs {
namespace sim {
using namespace boost::interprocess;

GuiClient::GuiClient(const std::string& id)
    : shm{.manager{open_only, id.c_str()},
          .state_lock{open_only, (id + STATE_LOCK_POSTFIX).c_str()},
          .info_lock{open_only, (id + INFO_LOCK_POSTFIX).c_str()}},
      id{id},
      m{nullptr},
      d{nullptr} {
  // setup shared memory
  std::tie(this->shm.info_byte, std::ignore) =
      this->shm.manager.find<bool>(INFO_BYTE);
  std::tie(this->shm.state.ptr, this->shm.state.size) =
      this->shm.manager.find<mjtNum>(STATE);
  std::tie(this->shm.model.ptr, this->shm.model.size) =
      this->shm.manager.find<char>(MODEL);
}

std::string GuiClient::get_model_bytes() const {
  return std::string(this->shm.model.ptr, this->shm.model.size);
}

void GuiClient::set_model_and_data(mjModel* m, mjData* d) {
  this->m = m;
  this->d = d;
}

void GuiClient::sync() {
  // Read new state
  this->shm.state_lock.lock_sharable();
  mj_setState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
  this->shm.state_lock.unlock_sharable();
  // Tell sim that we want new data
  this->shm.info_lock.lock();
  *this->shm.info_byte = true;
  this->shm.info_lock.unlock();
}
}  // namespace sim
}  // namespace rcs
