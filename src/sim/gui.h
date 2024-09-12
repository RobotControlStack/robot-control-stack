#include <boost/interprocess/interprocess_fwd.hpp>

#include "boost/interprocess/managed_shared_memory.hpp"
#include "platform_ui_adapter.h"
#include "sim.h"

namespace rcs {
namespace sim {
class GuiServer {
 public:
  GuiServer(Sim& sim, const std::string& id);
  ~GuiServer();

 private:
  Sim& sim;
  struct {
    struct {
      mjtNum* ptr;
      size_t size;
    } state;  // physics state in shared memory
    struct {
      char* ptr;
      size_t size;
    } model;          // binary model in shared memory
    char* info_byte;  // extra info byte in shared memory
    boost::interprocess::managed_shared_memory manager;
  } shm;
  void update_mjdata_callback();
};
class GuiClient {
 public:
  GuiClient(const std::string& id);
  ~GuiClient();
 private:
  mjModel* m;
  mjData* d;
  struct {
    struct {
      mjtNum* ptr;
      size_t size;
    } state;  // physics state in shared memory
    struct {
      char* ptr;
      size_t size;
    } model;          // binary model in shared memory
    char* info_byte;  // extra info byte in shared memory
    boost::interprocess::managed_shared_memory manager;
  } shm;
};
}  // namespace sim
}  // namespace rcs
