#ifndef RCS_FRANKA_CONTROL_STATE_BUFFER_H
#define RCS_FRANKA_CONTROL_STATE_BUFFER_H

#include <franka/errors.h>
#include <franka/robot_state.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rcs {
namespace hw {

/**
 * @brief One sample of the robot state as seen by the 1 kHz control loop.
 *
 * Plain old data so that it can be copied into a preallocated ring buffer
 * without any allocation inside the real-time loop.
 */
struct ControlStateSample {
  // host wall clock in seconds since epoch (same clock as Python's
  // time.time()), taken when the sample was recorded
  double timestamp = 0.0;
  // robot clock in seconds (franka::RobotState::time)
  double robot_time = 0.0;
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> tau_J{};
  std::array<double, 7> tau_J_d{};
  std::array<double, 7> dtau_J{};
  std::array<double, 7> tau_ext_hat_filtered{};
  std::array<double, 6> O_F_ext_hat_K{};
  std::array<double, 6> K_F_ext_hat_K{};
  // column major 4x4 end effector pose in base frame
  std::array<double, 16> O_T_EE{};
  double control_command_success_rate = 0.0;
  // franka::RobotMode as integer
  int32_t robot_mode = 0;
  // bit i is set if error i of franka::Errors is active
  uint64_t current_errors = 0;
  uint64_t last_motion_errors = 0;
};

// number of error flags in franka::Errors
constexpr size_t kNumFrankaErrors = 41;
static_assert(kNumFrankaErrors <= 64, "errors must fit into a uint64 mask");

/**
 * @brief Packs the error flags of franka::Errors into a bit mask.
 *
 * franka::Errors only exposes its flags as references into a private
 * std::array<bool, 41>, bound in declaration order. The first reference
 * therefore points to the start of that array.
 */
inline uint64_t ErrorsToMask(const franka::Errors& errors) noexcept {
  const bool* flags = &errors.joint_position_limits_violation;
  uint64_t mask = 0;
  for (size_t i = 0; i < kNumFrankaErrors; ++i) {
    if (flags[i]) {
      mask |= (uint64_t{1} << i);
    }
  }
  return mask;
}

/**
 * @brief Names of the error flags, index i corresponds to bit i of the masks.
 */
inline std::vector<std::string> ErrorNames() {
  std::vector<std::string> names;
  names.reserve(kNumFrankaErrors);
  for (size_t i = 0; i < kNumFrankaErrors; ++i) {
    std::array<bool, kNumFrankaErrors> flags{};
    flags[i] = true;
    // format is ["name"]
    std::string name;
    for (char c : static_cast<std::string>(franka::Errors(flags))) {
      if (c != '[' && c != ']' && c != '"') {
        name.push_back(c);
      }
    }
    names.push_back(name);
  }
  return names;
}

inline void FillSample(const franka::RobotState& state,
                       ControlStateSample& sample) noexcept {
  sample.timestamp = std::chrono::duration<double>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  sample.robot_time = state.time.toSec();
  sample.q = state.q;
  sample.dq = state.dq;
  sample.tau_J = state.tau_J;
  sample.tau_J_d = state.tau_J_d;
  sample.dtau_J = state.dtau_J;
  sample.tau_ext_hat_filtered = state.tau_ext_hat_filtered;
  sample.O_F_ext_hat_K = state.O_F_ext_hat_K;
  sample.K_F_ext_hat_K = state.K_F_ext_hat_K;
  sample.O_T_EE = state.O_T_EE;
  sample.control_command_success_rate = state.control_command_success_rate;
  sample.robot_mode = static_cast<int32_t>(state.robot_mode);
  sample.current_errors = ErrorsToMask(state.current_errors);
  sample.last_motion_errors = ErrorsToMask(state.last_motion_errors);
}

/**
 * @brief Single producer single consumer lock-free ring buffer of
 * ControlStateSample.
 *
 * The producer is the 1 kHz control thread (push), the consumer is the
 * Python env step (drain). All memory is allocated in the constructor, push
 * neither allocates nor locks. If the consumer does not drain in time the
 * newest samples are dropped and counted.
 */
class ControlStateBuffer {
 public:
  // 8192 samples are > 8 s at 1 kHz, much longer than one env step
  static constexpr size_t kDefaultCapacity = 8192;

  explicit ControlStateBuffer(size_t capacity = kDefaultCapacity)
      : m_capacity(RoundUpPow2(capacity)),
        m_mask(m_capacity - 1),
        m_samples(std::make_unique<ControlStateSample[]>(m_capacity)) {}

  ControlStateBuffer(const ControlStateBuffer&) = delete;
  ControlStateBuffer& operator=(const ControlStateBuffer&) = delete;

  size_t capacity() const noexcept { return m_capacity; }

  /// producer side, real-time safe
  bool push(const franka::RobotState& state) noexcept {
    const size_t head = m_head.load(std::memory_order_relaxed);
    const size_t tail = m_tail.load(std::memory_order_acquire);
    if (head - tail >= m_capacity) {
      m_dropped.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    FillSample(state, m_samples[head & m_mask]);
    m_head.store(head + 1, std::memory_order_release);
    return true;
  }

  /// consumer side: number of samples ready to be drained
  size_t size() const noexcept {
    return m_head.load(std::memory_order_acquire) -
           m_tail.load(std::memory_order_relaxed);
  }

  /**
   * @brief consumer side: calls fn(index, sample) for every available sample
   * in order and frees them. Returns the number of drained samples.
   */
  template <typename Fn>
  size_t drain(Fn&& fn, size_t max_samples) {
    const size_t tail = m_tail.load(std::memory_order_relaxed);
    const size_t head = m_head.load(std::memory_order_acquire);
    size_t n = head - tail;
    if (n > max_samples) {
      n = max_samples;
    }
    for (size_t i = 0; i < n; ++i) {
      fn(i, m_samples[(tail + i) & m_mask]);
    }
    m_tail.store(tail + n, std::memory_order_release);
    return n;
  }

  /// consumer side: discard all available samples
  void clear() noexcept {
    m_tail.store(m_head.load(std::memory_order_acquire),
                 std::memory_order_release);
  }

  /// number of samples dropped since the last call, resets the counter
  uint64_t take_dropped() noexcept {
    return m_dropped.exchange(0, std::memory_order_relaxed);
  }

 private:
  static size_t RoundUpPow2(size_t v) {
    size_t p = 1;
    while (p < v) {
      p <<= 1;
    }
    return p;
  }

  const size_t m_capacity;
  const size_t m_mask;
  std::unique_ptr<ControlStateSample[]> m_samples;
  // separate cache lines for producer and consumer indices
  alignas(64) std::atomic<size_t> m_head{0};
  alignas(64) std::atomic<size_t> m_tail{0};
  alignas(64) std::atomic<uint64_t> m_dropped{0};
};

}  // namespace hw
}  // namespace rcs

#endif  // RCS_FRANKA_CONTROL_STATE_BUFFER_H
