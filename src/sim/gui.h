#ifndef GUI_H
#define GUI_H
#include <mujoco/mujoco.h>

#include <bitset>
#include <optional>
#include <string>

#include "boost/interprocess/managed_shared_memory.hpp"

namespace rcs {
namespace sim {
enum {
  UPDATE_SIM_STATE = (1u << 0),
  CLIENT_OPEN = (1u << 1),
  SERVER_SHUTTING_DOWN = (1u << 2)
};
static const char* INFO_BYTE = "I";
static const char* STATE = "S";
static const char* MODEL = "M";
static constexpr mjtState MJ_PHYSICS_SPEC = mjSTATE_FULLPHYSICS;

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
void open_gui_window(std::string& id);

struct shm {
  struct {
    mjtNum* ptr = nullptr;
    size_t size = 0;
  } state;
  struct {
    char* ptr = nullptr;
    size_t size = 0;
  } model;
  char* info_byte = nullptr;  // extra info byte in shared memory
  boost::interprocess::managed_shared_memory manager;
};

class GuiServer {
 public:
  GuiServer(mjModel* m, mjData* d, const std::string& id);
  ~GuiServer();
  void update_mjdata_callback();

 private:
  // don't make this a reference, apparently references are not valid in
  // destructors. or at least in this case the reference to id was not longer
  // valid in the destructor.
  const std::string id;
  struct shm shm;
  mjModel* m;
  mjData* d;
};

}  // namespace sim
}  // namespace rcs
#endif
