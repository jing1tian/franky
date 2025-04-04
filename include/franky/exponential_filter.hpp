#pragma once

#include <optional>

#include <Eigen/Core>

namespace franky {

template <size_t dims>
class ExponentialFilter {
 public:
  explicit ExponentialFilter(double alpha) : alpha_(alpha) {
    if (alpha < 0 || alpha > 1)
      throw std::invalid_argument("alpha must be between 0 and 1");
  }

  ExponentialFilter(const ExponentialFilter&) = default;

  void add(const Eigen::Vector<double, dims>& value) {
    if (current_value_.has_value())
      current_value_ = alpha_ * current_value_.value() + (1 - alpha_) * value;
    else
      current_value_ = value;
  }

  Eigen::Vector<double, dims> current_value() const { return current_value_.value(); }

  Eigen::Vector<double, dims> operator()(const Eigen::Vector<double, dims>& value) {
    add(value);
    return current_value();
  }

 private:
  std::optional<Eigen::Vector<double, dims>> current_value_;
  double alpha_;
};

}  // namespace franky
