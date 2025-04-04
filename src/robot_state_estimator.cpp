#include "franky/robot_state_estimator.hpp"

#include <franky/model.hpp>
#include <franky/robot_state.hpp>

namespace franky {
RobotStateEstimator::RobotStateEstimator(
    size_t dq_window_size, size_t ddq_window_size, double dq_alpha, double ddq_alpha)
    : dq_median_filter_(dq_window_size),
      dq_exponential_filter_(dq_alpha),
      ddq_median_filter_(ddq_window_size),
      ddq_exponential_filter_(ddq_alpha),
      prev_time_(std::nullopt) {}

RobotState RobotStateEstimator::update(const franka::RobotState& franka_robot_state, const Model& model) {
  Vector7d prev_dq_est;
  if (prev_time_.has_value()) prev_dq_est = dq_exponential_filter_.current_value();

  auto dq_est = dq_exponential_filter_(dq_median_filter_(toEigenD(franka_robot_state.dq)));

  Vector7d ddq_est_raw;
  if (prev_time_.has_value()) {
    ddq_est_raw = (dq_est - prev_dq_est) / (franka_robot_state.time - prev_time_.value()).toSec();
  } else {
    ddq_est_raw = toEigenD(franka_robot_state.ddq_d);
  }

  auto ddq_est = ddq_exponential_filter_(ddq_median_filter_(ddq_est_raw));

  auto ee_jacobian = model.bodyJacobian(
      franka::Frame::kEndEffector,
      toEigenD(franka_robot_state.q),
      stdToAffine(franka_robot_state.F_T_EE),
      stdToAffine(franka_robot_state.EE_T_K));
  prev_time_ = franka_robot_state.time;
  return RobotState::from_franka(franka_robot_state, ee_jacobian, dq_est, ddq_est);
}

}  // namespace franky