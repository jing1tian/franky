#include "franky/dynamics_limit.hpp"

#include <array>

#include "franky/types.hpp"
#include "franky/util.hpp"

namespace franky {

template <>
void DynamicsLimit<double>::check(const double &value) {
  if (value < 0) {
    std::stringstream ss;
    ss << desc << " limit cannot be negative.";
    throw std::runtime_error(ss.str());
  }
  if (value > max) {
    std::stringstream ss;
    ss << desc << " limit (" << value << ") cannot be greater than the maximum value (" << max << ").";
    throw std::runtime_error(ss.str());
  }
}

template <>
template <>
void DynamicsLimit<std::array<double, 7>>::setFrom<Array<7>>(const Array<7> &value) {
  set(ensureStd<7>(value));
}

template <>
template <>
Vector7d DynamicsLimit<std::array<double, 7>>::getAs<Vector7d>() {
  return Vector7d::Map(value_.data());
}

template <>
void DynamicsLimit<std::array<double, 7>>::check(const std::array<double, 7> &value) {
  for (int i = 0; i < value.size(); i++) {
    if (value[i] < 0) {
      std::stringstream ss;
      ss << desc << " limit cannot be negative.";
      throw std::runtime_error(ss.str());
    }
    if (value[i] > max[i]) {
      std::stringstream ss;
      ss << desc << " limit at index " << i << "(" << value[i] << ") cannot be greater than the maximum value ("
         << max[i] << ").";
      throw std::runtime_error(ss.str());
    }
  }
}

}  // namespace franky
