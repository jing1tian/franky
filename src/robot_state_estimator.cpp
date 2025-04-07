#include "franky/robot_state_estimator.hpp"

#include <franky/model.hpp>
#include <franky/robot_state.hpp>

namespace franky {
RobotStateEstimator::RobotStateEstimator(
    double q_process_var, double dq_process_var, double ddq_process_var, double q_obs_var, double dq_obs_var,
    double q_d_obs_var, double dq_d_obs_var, double ddq_d_obs_var)
    : q_process_var_(q_process_var),
      dq_process_var_(dq_process_var),
      ddq_process_var_(ddq_process_var),
      prev_time_(std::nullopt) {
  const Eigen::Matrix<double, 7, 7> one = Eigen::Matrix<double, 7, 7>::Identity();
  const Eigen::Matrix<double, 7, 7> zero = Eigen::Matrix<double, 7, 7>::Zero();

  // State format: q * 7, dq * 7, ddq * 7
  // Observation format: q * 7, dq * 7, q_d * 7, dq_d * 7, ddq_d * 7

  // clang-format off
  h_mat_ << one, zero, zero,
            zero, one, zero,
            one, zero, zero,
            zero, one, zero,
            zero, zero, one;

  r_mat_ << one * q_obs_var, zero,             zero,              zero,               zero,
            zero,            one * dq_obs_var, zero,              zero,               zero,
            zero,            zero,             one * q_d_obs_var, zero,               zero,
            zero,            zero,             zero,              one * dq_d_obs_var, zero,
            zero,            zero,             zero,              zero,               one * ddq_d_obs_var;
  // clang-format on
}

RobotState RobotStateEstimator::update(const franka::RobotState& franka_robot_state, const Model& model) {
  const Eigen::Matrix<double, 7, 7> one = Eigen::Matrix<double, 7, 7>::Identity();
  const Eigen::Matrix<double, 7, 7> zero = Eigen::Matrix<double, 7, 7>::Zero();

  const auto q = toEigenD(franka_robot_state.q);
  const auto dq = toEigenD(franka_robot_state.dq);
  const auto q_d = toEigenD(franka_robot_state.q_d);
  const auto dq_d = toEigenD(franka_robot_state.dq_d);
  const auto ddq_d = toEigenD(franka_robot_state.ddq_d);

  Eigen::Vector<double, 5 * 7> observation;
  observation << q, dq, q_d, dq_d, ddq_d;

  if (prev_time_.has_value()) {
    const auto dt = (franka_robot_state.time - prev_time_.value()).toSec();

    // clang-format off
    Eigen::Matrix<double, 3 * 7, 3 * 7> f_mat;
    f_mat << one,   one * dt, one * 0.5 * std::pow(dt, 2),
             zero,  one,      one * dt,
             zero,  zero,     one;

    Eigen::Matrix<double, 3 * 7, 7> g_mat_acc;
    g_mat_acc << one * 0.5 * std::pow(dt, 2),
                 one * dt,
                 one;

    Eigen::Matrix<double, 3 * 7, 7> g_mat_vel;
    g_mat_vel << one * dt,
                 one,
                 zero;

    Eigen::Matrix<double, 3 * 7, 7> g_mat_pos;
    g_mat_pos << one,
                 zero,
                 zero;
    // clang-format on
    const auto q_mat = q_process_var_ * (g_mat_pos * g_mat_pos.transpose()) +
                       dq_process_var_ * (g_mat_vel * g_mat_vel.transpose()) +
                       ddq_process_var_ * (g_mat_acc * g_mat_acc.transpose());

    // Predict
    joint_state_mean_ = f_mat * joint_state_mean_;
    joint_state_covar_ = f_mat * joint_state_covar_ * f_mat.transpose() + q_mat;

    // Observe
    const auto s_mat = h_mat_ * joint_state_covar_ * h_mat_.transpose() + r_mat_;
    const auto k_mat = joint_state_covar_ * h_mat_.transpose() * s_mat.inverse();

    const auto observation_error = observation - h_mat_ * joint_state_mean_;
    joint_state_mean_ = joint_state_mean_ + k_mat * observation_error;
    joint_state_covar_ = (Eigen::Matrix<double, 3 * 7, 3 * 7>::Identity() - k_mat * h_mat_) * joint_state_covar_;
  } else {
    // Use MAP estimate
    auto r_mat_inv = r_mat_.inverse();
    joint_state_covar_ = (h_mat_.transpose() * r_mat_inv * h_mat_).inverse();
    joint_state_mean_ = joint_state_covar_ * h_mat_.transpose() * r_mat_inv * observation;
  }

  const auto ee_jacobian = model.bodyJacobian(
      franka::Frame::kEndEffector,
      toEigenD(franka_robot_state.q),
      stdToAffine(franka_robot_state.F_T_EE),
      stdToAffine(franka_robot_state.EE_T_K));
  prev_time_ = franka_robot_state.time;
  return RobotState::from_franka(
      franka_robot_state,
      ee_jacobian,
      joint_state_mean_.segment<7>(0),
      joint_state_mean_.segment<7>(7),
      joint_state_mean_.segment<7>(14));
}

}  // namespace franky