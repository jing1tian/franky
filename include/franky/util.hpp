#pragma once

#include <array>
#include <Eigen/Core>

namespace franky {

template<size_t dims>
std::array<double, dims> toStd(const Eigen::Matrix<double, dims, 1> &vector) {
  std::array<double, dims> result;
  Eigen::Matrix<double, dims, 1>::Map(result.data()) = vector;
  return result;
}

template<size_t dims, typename T>
Eigen::Matrix<T, dims, 1> toEigen(const std::array<T, dims> &vector) {
  return Eigen::Matrix<T, dims, 1>::Map(vector.data());
}

template<size_t dims>
Eigen::Matrix<double, dims, 1> toEigenD(const std::array<double, dims> &vector) {
  return Eigen::Matrix<double, dims, 1>::Map(vector.data());
}

template<size_t dims>
std::array<double, dims> ensureStd(const Array<dims> &input) {
  if (std::holds_alternative<std::array<double, dims >>(input))
    return std::get<std::array<double, dims >>(input);
  return toStd<dims>(std::get<Eigen::Vector<double, dims >>(input));
}

template<size_t dims>
std::array<double, dims> expand(const ScalarOrArray<dims> &input) {
  if (std::holds_alternative<Array<dims>>(input)) {
    return ensureStd<dims>(std::get<Array<dims>>(input));
  }
  std::array<double, dims> output;
  std::fill(output.begin(), output.end(), std::get<double>(input));
  return output;
}

}  // namespace franky
