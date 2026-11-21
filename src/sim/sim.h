#ifndef RCS_SIM_H
#define RCS_SIM_H
#include <mujoco/mjvisualize.h>

#include <functional>
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
// Both renderers are implemented in python, see python/rcs/camera/sim.py
enum class RendererBackend { CLASSIC, FILAMENT };

struct SimConfig {
  bool async_control = false;
  bool realtime = false;
  double frequency = 30;  // in Hz
  int max_convergence_steps =
      1000;  // physics steps, 1 s at the default timestep
  // MuJoCo arena (stack + constraint) size in bytes, applied when a scene is
  // compiled from a ModelComposer. -1 keeps the model's <size memory> or
  // MuJoCo's automatic sizing, which can be too small for scenes with many
  // contacts ("mj_stackAlloc: insufficient memory").
  long long memory = -1;
  RendererBackend renderer = RendererBackend::CLASSIC;
  // physics timestep in seconds, replaces the one of the model; torque
  // control runs the controllers once per step, as the 1 ms loop of the FR3
  double timestep = 0.001;
};

/* Fixed schedule of a callback in simulation time: the k-th call is due at
 * start + k * period and happens at the physics step closest to that time, so
 * that the calls do not drift. The schedule restarts on reset and whenever
 * the simulation time goes backwards. */
struct CallbackSchedule {
  mjtNum period;               // in seconds, 0 calls the callback every step
  mjtNum next_call_time = 0;   // in seconds of simulation time
  mjtNum last_check_time = 0;  // to detect the simulation time going backwards
};

struct Callback {
  std::function<void(void)> cb;
  // called when the schedule (re)starts: at registration, on reset and when
  // the simulation time goes backwards, may be empty
  std::function<void(void)> on_restart;
  CallbackSchedule schedule;
};

struct ConditionCallback {
  std::function<bool(void)> cb;
  CallbackSchedule schedule;
  bool last_return_value = false;
};

/* Simulation states captured at a fixed rate of simulation time, e.g. at the
 * exposure times of a camera, so that they can be rendered later. Kept in a
 * ring buffer of the most recent states. The captures are taken by a
 * callback. */
struct StateCapture {
  size_t callback_id;
  int state_spec;  // mjtState bit mask passed to mj_getState
  int state_size;
  size_t capacity;
  std::vector<mjtNum> states;      // capacity x state_size, row major
  std::vector<mjtNum> timestamps;  // capacity
  size_t next_index = 0;
  size_t size = 0;
};

struct CapturedState {
  mjtNum timestamp;
  rcs::common::VectorXd state;
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
  std::vector<StateCapture> state_captures;
  std::vector<DynamicJointSpec> dynamic_joint_specs;
  std::unordered_map<std::string, size_t> dynamic_joint_name_to_index;
  enum class Due { no, yes, restarted };
  Due poll(CallbackSchedule& schedule);
  void restart(CallbackSchedule& schedule);
  void restart(Callback& callback);
  void invoke_callbacks();
  bool invoke_condition_callbacks();
  void capture_state(size_t id);
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
  /* Callbacks are called after each physics step that is closest to their
   * fixed schedule of start + k * seconds_between_calls in simulation time,
   * i.e. the actual calls jitter by up to half a timestep but do not drift.
   * The schedule starts at registration and restarts on reset and whenever
   * the simulation time goes backwards. seconds_between_calls = 0 calls the
   * callback after every step. Condition callbacks are only evaluated in
   * step_until_convergence. */
  size_t register_cb(std::function<void(void)> cb, mjtNum seconds_between_calls,
                     std::function<void(void)> on_restart = nullptr);
  void register_any_cb(std::function<bool(void)> cb,
                       mjtNum seconds_between_calls);
  void register_all_cb(std::function<bool(void)> cb,
                       mjtNum seconds_between_calls);
  /* Captures the simulation state (mj_getState with state_spec) frame_rate
   * times per second of simulation time and keeps the latest buffer_size
   * states. The current state is captured right away. The captures are taken
   * by a callback, i.e. they follow its fixed schedule and happen after
   * integration so that the state is consistent with its time stamp. Returns
   * the id of the capture. */
  size_t register_state_capture(double frame_rate, size_t buffer_size,
                                int state_spec);
  /* Latest captured state with a time stamp <= max_timestamp */
  std::optional<CapturedState> get_captured_state(size_t id,
                                                  mjtNum max_timestamp) const;
  size_t captured_state_count(size_t id) const;
  /* Clears the buffer and restarts the capture schedule at the current state */
  void clear_captured_states(size_t id);
  void start_gui_server(const std::string& id);
  void stop_gui_server();
  void sync_gui();
};
}  // namespace sim
}  // namespace rcs
#endif
