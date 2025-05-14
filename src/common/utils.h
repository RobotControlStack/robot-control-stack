#ifndef RCS_UTIL_H
#define RCS_UTIL_H

#include <eigen3/Eigen/Eigen>
#include <memory>

namespace rcs {
namespace common {

typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;
typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;
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

}  // namespace common
}  // namespace rcs

#endif  // RCS_UTIL_H
