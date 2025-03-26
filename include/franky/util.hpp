#pragma once

#include <array>
#include <Eigen/Core>

namespace franky {

template<size_t dims>
inline std::array<double, dims> toStd(const Eigen::Matrix<double, dims, 1> &vector) {
  std::array<double, dims> result;
  Eigen::Matrix<double, dims, 1>::Map(result.data()) = vector;
  return result;
}

template<size_t dims, typename T>
inline Eigen::Matrix<T, dims, 1> toEigen(const std::array<T, dims> &vector) {
  return Eigen::Matrix<T, dims, 1>::Map(vector.data());
}

template<size_t dims>
inline Eigen::Matrix<double, dims, 1> toEigenD(const std::array<double, dims> &vector) {
  return Eigen::Matrix<double, dims, 1>::Map(vector.data());
}

}  // namespace franky
