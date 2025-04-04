#pragma once

#include <Eigen/Dense>
#include <algorithm>

namespace franky {

template <size_t dims>
class MedianFilter {
 public:
  explicit MedianFilter(size_t window_size)
      : window_size_(window_size),
        current_index_(0),
        current_size_(0),
        window_(Eigen::Matrix<double, dims, Eigen::Dynamic>::Zero(dims, window_size)) {
    if (window_size <= 0) throw std::invalid_argument("Window size must be positive.");
  }

  MedianFilter(const MedianFilter&) = default;

  void add(const Eigen::Vector<double, dims>& value) {
    window_.col(current_index_) = value;
    current_index_ = (current_index_ + 1) % window_size_;
    if (current_size_ < window_size_) ++current_size_;
  }

  Eigen::Vector<double, dims> current_value() const {
    Eigen::Vector<double, dims> median;

    for (size_t d = 0; d < dims; ++d) {
      Eigen::VectorXd values = window_.row(d).head(current_size_).transpose();
      std::nth_element(values.data(), values.data() + current_size_ / 2, values.data() + current_size_);
      median(d) = values(current_size_ / 2);
    }

    return median;
  }

  Eigen::Vector<double, dims> operator()(const Eigen::Vector<double, dims>& value) {
    add(value);
    return current_value();
  }

 private:
  size_t window_size_;
  size_t current_index_;
  size_t current_size_;
  Eigen::Matrix<double, dims, Eigen::Dynamic> window_;
};

}  // namespace franky
