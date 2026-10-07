#ifndef RCS_SIM_H
#define RCS_SIM_H
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "boost/interprocess/managed_shared_memory.hpp"
#include "gui.h"
#include "mujoco/mujoco.h"
#include "rcs/utils.h"

namespace rcs {
namespace sim {
// Cameras are rendered on the Python side (rcs.camera.sim) with either
// MuJoCo's classic OpenGL renderer or Filament.
enum class RendererBackend { CLASSIC, FILAMENT };

struct SimConfig {
  bool async_control = false;
  bool realtime = false;
  double frequency = 30;  // in Hz
  int max_convergence_steps = 500;
  RendererBackend renderer = RendererBackend::CLASSIC;
};

struct Callback {
  const std::function<void(void)> cb;
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;
};

struct ConditionCallback {
  const std::function<bool(void)> cb;
  mjtNum seconds_between_calls;  // in seconds
  mjtNum last_call_timestamp;    // in seconds
  bool last_return_value;
};

struct StateSnapshot {
  mjtNum time;
  rcs::common::VectorXd state;  // mj_getState with mjSTATE_INTEGRATION
};

// Records the simulation state at a fixed rate while stepping. Cameras render
// lazily from the latest snapshot of their frame rate, which gives realistic
// camera timing without rendering frames nobody reads (see
// docs/development/camera_snapshot_rendering.md).
class StateSnapshotter {
 public:
  explicit StateSnapshotter(mjtNum seconds_between_snapshots);
  // takes a snapshot if a period has been crossed since the last one
  void update(const mjModel* m, const mjData* d);
  std::optional<StateSnapshot> latest() const;
  void reset();

 private:
  mjtNum seconds_between_snapshots;
  mjtNum last_snapshot_time;
  std::optional<StateSnapshot> snapshot;
  mutable std::mutex mutex;
};

struct DynamicJointSchema {
  std::vector<std::string> joint_names;
  std::vector<int> joint_types;
  std::vector<int> qpos_sizes;
  std::vector<int> qvel_sizes;
};

struct DynamicJointState {
  rcs::common::VectorXd qpos;
  rcs::common::VectorXd qvel;
};

class Sim {
 private:
  struct DynamicJointSpec {
    std::string name;
    int type;
    int qpos_adr;
    int qvel_adr;
    int qpos_size;
    int qvel_size;
  };

  SimConfig cfg;
  std::vector<Callback> callbacks;
  std::vector<ConditionCallback> any_callbacks;
  std::vector<ConditionCallback> all_callbacks;
  // keyed by frame rate in Hz
  std::map<int, std::unique_ptr<StateSnapshotter>> state_snapshotters;
  std::vector<DynamicJointSpec> dynamic_joint_specs;
  std::unordered_map<std::string, size_t> dynamic_joint_name_to_index;
  void invoke_callbacks();
  bool invoke_condition_callbacks();
  void update_state_snapshots();
  void init_dynamic_joint_specs();
  static int get_joint_qpos_size(int joint_type);
  static int get_joint_qvel_size(int joint_type);
  size_t convergence_steps = 0;
  bool converged = true;
  std::optional<GuiServer> gui;
  bool gui_callback_registered = false;

 public:
  // TODO: hide m & d, pass as parameter to callback (easier refactoring)
  mjModel* m;
  mjData* d;
  Sim(mjModel* m, mjData* d);
  bool set_config(const SimConfig& cfg);
  SimConfig get_config();
  bool is_converged();
  void step_until_convergence();
  void step(size_t k);
  void reset_callbacks();
  void reset();
  DynamicJointSchema get_dynamic_joint_schema() const;
  DynamicJointState get_dynamic_joint_state() const;
  void set_dynamic_joint_state(const DynamicJointSchema& schema,
                               const DynamicJointState& state);
  /* NOTE: IMPORTANT, the callback is not necessarily called at exactly the
   * the requested interval. We invoke a callback if the elapsed simulation time
   * since the last call of the callback is greater than the requested time.
   * The exact interval at which they are called depends on the models timestep
   * option (cf.
   * https://mujoco.readthedocs.io/en/stable/programming/simulation.html#simulation-loop
   */
  void register_cb(std::function<void(void)> cb, mjtNum seconds_between_calls);
  void register_any_cb(std::function<bool(void)> cb,
                       mjtNum seconds_between_calls);
  void register_all_cb(std::function<bool(void)> cb,
                       mjtNum seconds_between_calls);
  // Records the state at the given frame rate while stepping; registering the
  // same rate twice is a no-op. Snapshots are cleared by reset().
  void register_state_snapshots(int frame_rate);
  std::optional<StateSnapshot> get_latest_state_snapshot(int frame_rate) const;
  void start_gui_server(const std::string& id);
  void stop_gui_server();
  void sync_gui();
};
}  // namespace sim
}  // namespace rcs
#endif
