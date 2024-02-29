#ifndef RSC_UTIL_H
#define RSC_UTL_H

#include <eigen3/Eigen/Eigen>
#include <memory>

namespace rcs {
namespace common {

// convert between eigen and array
template <auto N, auto M>
std::array<double, N * M> eigen2array(
    Eigen::Matrix<double, N, M, Eigen::ColMajor> matrix) {
  std::array<double, N * M> array;
  Eigen::Matrix<double, N, M>::Map(array.data()) = matrix;
}

// convert between array and eigen
template <auto N, auto M>
Eigen::Matrix<double, N, M, Eigen::ColMajor> array2eigen(
    std::array<double, N * M> array) {
  Eigen::Matrix<double, N, M, Eigen::ColMajor> matrix(array.data());
  return matrix;
}

}  // namespace common
}  // namespace rcs
#endif  // RSC_UTL_H