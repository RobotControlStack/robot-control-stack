#ifndef RCS_UTIL_H
#define RCS_UTIL_H

#include <Eigen/Eigen>
#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace rcs {
namespace common {

typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;
typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;
typedef Eigen::Matrix<double, 16, 1, Eigen::ColMajor> Vector16d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor> VectorXd;
typedef Eigen::Matrix<int, 7, 1, Eigen::ColMajor> Vector7i;

/***
 * @brief convert between eigen and flattened array (col major as required by
 * libfranka)
 */
template <auto N, auto M>
std::array<double, N * M> eigen2array(
    Eigen::Matrix<double, N, M, Eigen::ColMajor> matrix) {
  std::array<double, N * M> array;
  Eigen::Matrix<double, N, M>::Map(array.data()) = matrix;
  return array;
}

/***
 * @brief convert between flattened array (col major as required by libfranka)
 * and eigen
 */
template <auto N, auto M>
Eigen::Matrix<double, N, M, Eigen::ColMajor> array2eigen(
    std::array<double, N * M> array) {
  Eigen::Matrix<double, N, M, Eigen::ColMajor> matrix(array.data());
  return matrix;
}
void bootstrap_egl(std::uintptr_t fn_addr, std::uintptr_t display,
                   std::uintptr_t context);
void ensure_current();

/***
 * @brief thread safe holder for a single value, e.g. to hand data from a
 * non-realtime thread to a control loop
 */
template <typename T>
class ThreadSafeValue {
 private:
  T value_;
  mutable std::mutex mutex_;

 public:
  ThreadSafeValue() = default;
  explicit ThreadSafeValue(const T& value) : value_(value) {}

  void store(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
  }

  T load() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  /***
   * @brief atomically read the value and reset it to a default constructed one
   */
  T load_and_clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::exchange(value_, T{});
  }
};

/***
 * @brief thread safe ring buffer of fixed maximum size, drops the oldest
 * element once the buffer is full
 */
template <typename T>
class ThreadSafeFixedBuffer {
 private:
  std::deque<T> deque_;
  size_t max_size_;
  mutable std::mutex mutex_;

 public:
  explicit ThreadSafeFixedBuffer(size_t max_size) : max_size_(max_size) {}

  void push_back(const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);

    deque_.push_back(value);
    if (deque_.size() > max_size_) {
      deque_.pop_front();
    }
  }

  T get(size_t index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return deque_[index];
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return deque_.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    deque_.clear();
  }

  // Copy of the newest ``n`` elements (oldest first). Bounded cost for
  // readers that must not copy the whole buffer (e.g. a 1 kHz control tick).
  std::vector<T> last_n(size_t n) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t count = std::min(n, deque_.size());
    return std::vector<T>(deque_.end() - static_cast<long>(count), deque_.end());
  }

  std::vector<T> to_vector() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::vector<T>(deque_.begin(), deque_.end());
  }
};

}  // namespace common
}  // namespace rcs

#endif  // RCS_UTIL_H
