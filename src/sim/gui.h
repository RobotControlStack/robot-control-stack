#include <boost/interprocess/interprocess_fwd.hpp>
#include <optional>

#include "boost/interprocess/managed_shared_memory.hpp"
#include "platform_ui_adapter.h"
#include "sim.h"

namespace rcs {
namespace sim {
class GuiBase {
 protected:
  struct {
    struct {
      mjtNum* ptr = nullptr;
      size_t size;
    } state;  // physics state in shared memory
    struct {
      char* ptr = nullptr;
      size_t size;
    } model;                    // binary model in shared memory
    char* info_byte = nullptr;  // extra info byte in shared memory
    boost::interprocess::managed_shared_memory manager;
  } shm;
  const std::string& id;
  GuiBase(boost::interprocess::managed_shared_memory manager,
          const std::string& id,
          std::optional<size_t> state_size = 0,
          std::optional<size_t> model_size = 0);
};

class GuiServer : GuiBase {
 public:
  GuiServer(Sim& sim, const std::string& id);
  ~GuiServer();

 private:
  Sim& sim;
  void update_mjdata_callback();
};
class GuiClient : GuiBase {
 public:
  GuiClient(const std::string& id);
  ~GuiClient();

 private:
  mjModel* m;
  mjData* d;
};
}  // namespace sim
}  // namespace rcs
