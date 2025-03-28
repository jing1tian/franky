#pragma once

#include <array>
#include <variant>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

namespace franky {

using Vector6d = Eigen::Vector<double, 6>;
using Vector7d = Eigen::Vector<double, 7>;

using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

using Affine = Eigen::Affine3d;

template<size_t dims>
using Array = std::variant<std::array<double, dims>, Eigen::Vector<double, dims>>;

template<size_t dims>
using ScalarOrArray = std::variant<double, Array<dims>>;

}