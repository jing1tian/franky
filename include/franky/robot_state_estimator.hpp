#pragma once

#include "franky/exponential_filter.hpp"
#include "franky/median_filter.hpp"
#include "franky/robot_state.hpp"
#include "franky/model.hpp"

namespace franky {

class RobotStateEstimator {
 public:
  RobotStateEstimator(size_t dq_window_size, size_t ddq_window_size, double dq_alpha, double ddq_alpha);

  RobotStateEstimator(const RobotStateEstimator&) = default;

  RobotState update(const franka::RobotState& franka_robot_state, const Model& model);

  RobotState operator()(const franka::RobotState& franka_robot_state, const Model& model) {
    return update(franka_robot_state, model);
  }

 private:
  MedianFilter<7> dq_median_filter_;
  ExponentialFilter<7> dq_exponential_filter_;
  MedianFilter<7> ddq_median_filter_;
  ExponentialFilter<7> ddq_exponential_filter_;

  std::optional<franka::Duration> prev_time_{};
};

}  // namespace franky
