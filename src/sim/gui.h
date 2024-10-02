#ifndef GUI_H
#define GUI_H
#include <mujoco/mujoco.h>

#include <bitset>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_sharable_mutex.hpp>
#include <boost/interprocess/sync/named_upgradable_mutex.hpp>
#include <optional>
#include <string>

namespace rcs {
namespace sim {
using namespace boost::interprocess;
static const char* INFO_BYTE = "I";
static const char* STATE = "S";
static const char* MODEL = "M";
static const std::string STATE_LOCK_POSTFIX = "_state_lock";
static const std::string INFO_LOCK_POSTFIX = "_info_lock";
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
  bool* info_byte = nullptr;  // extra info byte in shared memory
  managed_shared_memory manager;
  named_sharable_mutex state_lock;
  named_upgradable_mutex info_lock;
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
