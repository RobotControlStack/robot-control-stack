#include "gui.h"

#include <mujoco/mjdata.h>

#include <boost/interprocess/creation_tags.hpp>
#include <boost/interprocess/interprocess_fwd.hpp>

const char* INFO_BYTE = "I";
const char* STATE = "S";
const char* MODEL = "M";
constexpr mjtState MJ_PHYSICS_SPEC = mjSTATE_FULLPHYSICS;

static size_t calculate_state_size(mjModel const* m) {
  return mj_stateSize(m, MJ_PHYSICS_SPEC);
}

static size_t calculate_mdl_size(mjModel const* m) { return mj_sizeModel(m); }

static size_t calculate_shm_size(mjModel const* m, mjData const* d) {
  size_t const extra_info_size = 1;
  size_t const total_required_size = extra_info_size + calculate_mdl_size(m) +
                                     calculate_state_size(m) * sizeof(mjtNum);
  size_t const estimated_overhead =
      total_required_size * 0.1;  // 10% overhead estimation
  return total_required_size + estimated_overhead;
}

enum {
  UPDATE_SIM_STATE = (1u << 0),
  CLIENT_OPEN = (1u << 1),
  SERVER_SHUTTING_DOWN = (1u << 2)
};

namespace rcs {
namespace sim {

GuiBase::GuiBase(boost::interprocess::managed_shared_memory manager,
                 std::optional<size_t> state_size,
                 std::optional<size_t> model_size)
    : shm{.state{.size = state_size.value()},
          .model{.size = model_size.value()},
          .manager{std::move(manager)}} {}

GuiServer::GuiServer(Sim& sim, const std::string& id)
    : sim{sim},
      GuiBase(boost::interprocess::managed_shared_memory(
                  boost::interprocess::create_only, id.c_str(),
                  calculate_shm_size(sim.m, sim.d)),
              calculate_state_size(sim.m), calculate_mdl_size(sim.m)) {
  this->shm.state.ptr =
      this->shm.manager.construct<mjtNum>(STATE)[this->shm.state.size]();
  this->shm.model.ptr =
      this->shm.manager.construct<char>(MODEL)[this->shm.model.size]();
  this->shm.info_byte = this->shm.manager.construct<char>(INFO_BYTE)(0);
  mj_step(sim.m, sim.d);  // TODO: check if this is necessary
  mj_getState(sim.m, sim.d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
  mj_saveModel(sim.m, NULL, this->shm.model.ptr, this->shm.model.size);
}

GuiServer::~GuiServer(){
    // TODO: set server shutting down, wait until client open is false, then
    // delete shared memory
    // TODO: implement callback unergister, then unregister the callback.
};

void GuiServer::update_mjdata_callback() {
  if (*this->shm.info_byte & UPDATE_SIM_STATE) {
    mj_getState(this->sim.m, this->sim.d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
    *this->shm.info_byte &= ~UPDATE_SIM_STATE;
  }
}

GuiClient::GuiClient(const std::string& id)
    : GuiBase(boost::interprocess::managed_shared_memory(
          boost::interprocess::open_only, id.c_str())) {
  std::tie(this->shm.info_byte, std::ignore) =
      this->shm.manager.find<char>(INFO_BYTE);
  std::tie(this->shm.state.ptr, this->shm.state.size) =
      this->shm.manager.find<mjtNum>(STATE);
  mjVFS* vfs = (mjVFS*)mju_malloc(sizeof(mjVFS));
  mj_defaultVFS(vfs);
  int failed = mj_addBufferVFS(vfs, "model.mjb", this->shm.model.ptr,
                               this->shm.model.size);
  if (failed != 0) {
    // TODO: error handling & cleanup VFS
  }
  // TODO: cleanup VFS
  this->m = mj_loadModel("model.mjb", vfs);
  this->d = mj_makeData(m);
  mj_setState(this->m, this->d, this->shm.state.ptr, MJ_PHYSICS_SPEC);
  mj_step(this->m, this->d);
  // TODO: setup GUI with platform ui adapter
}
}  // namespace sim
}  // namespace rcs
