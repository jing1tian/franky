#include "franky/robot.hpp"

#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"
#include "franky/types.hpp"

namespace franky {

//! Connects to a robot at the given FCI IP address.
Robot::Robot(const std::string &fci_hostname) : Robot(fci_hostname, Params()) {}

Robot::Robot(const std::string &fci_hostname, const Params &params)
    : fci_hostname_(fci_hostname), params_(params), franka::Robot(fci_hostname, params.realtime_config) {
  setCollisionBehavior(params_.default_torque_threshold, params_.default_force_threshold);
}

bool Robot::hasErrors() {
  return bool(state().current_errors);
}

bool Robot::recoverFromErrors() {
  automaticErrorRecovery();
  return !hasErrors();
}

JointState Robot::currentJointState() {
  auto s = state();
  return {Eigen::Map<const Vector7d>(state().q.data()), Eigen::Map<const Vector7d>(state().dq.data())};
}

Vector7d Robot::currentJointPositions() {
  return currentJointState().position();
}

Vector7d Robot::currentJointVelocities() {
  return currentJointState().velocity();
}

Affine Robot::forwardKinematics(const Vector7d &q) {
  return Kinematics::forward(q);
}

franka::RobotState Robot::state() {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  {
    std::lock_guard<std::mutex> control_lock(control_mutex_);
    if (!is_in_control_unsafe()) {
      current_state_ = readOnce();
    }
  }
  return current_state_;
}

void Robot::setCollisionBehavior(const ScalarOrArray<7> &torque_threshold, const ScalarOrArray<6> &force_threshold) {
  setCollisionBehavior(torque_threshold, torque_threshold, force_threshold, force_threshold);
}

void Robot::setCollisionBehavior(
    const ScalarOrArray<7> &lower_torque_threshold,
    const ScalarOrArray<7> &upper_torque_threshold,
    const ScalarOrArray<6> &lower_force_threshold,
    const ScalarOrArray<6> &upper_force_threshold) {
  franka::Robot::setCollisionBehavior(
      expand<7>(lower_torque_threshold),
      expand<7>(upper_torque_threshold),
      expand<6>(lower_force_threshold),
      expand<6>(upper_force_threshold));
}

void Robot::setCollisionBehavior(
    const ScalarOrArray<7> &lower_torque_threshold_acceleration,
    const ScalarOrArray<7> &upper_torque_threshold_acceleration,
    const ScalarOrArray<7> &lower_torque_threshold_nominal,
    const ScalarOrArray<7> &upper_torque_threshold_nominal,
    const ScalarOrArray<6> &lower_force_threshold_acceleration,
    const ScalarOrArray<6> &upper_force_threshold_acceleration,
    const ScalarOrArray<6> &lower_force_threshold_nominal,
    const ScalarOrArray<6> &upper_force_threshold_nominal) {
  franka::Robot::setCollisionBehavior(
      expand<7>(lower_torque_threshold_acceleration),
      expand<7>(upper_torque_threshold_acceleration),
      expand<7>(lower_torque_threshold_nominal),
      expand<7>(upper_torque_threshold_nominal),
      expand<6>(lower_force_threshold_acceleration),
      expand<6>(upper_force_threshold_acceleration),
      expand<6>(lower_force_threshold_nominal),
      expand<6>(upper_force_threshold_nominal));

}

bool Robot::is_in_control_unsafe() const {
  return motion_generator_running_;
}

bool Robot::is_in_control() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  return is_in_control_unsafe();
}

std::string Robot::fci_hostname() const {
  return fci_hostname_;
}

std::optional<ControlSignalType> Robot::current_control_signal_type() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  if (!is_in_control_unsafe())
    return std::nullopt;
  if (std::holds_alternative<MotionGenerator<franka::Torques>>(motion_generator_))
    return ControlSignalType::Torques;
  else if (std::holds_alternative<MotionGenerator<franka::JointVelocities>>(motion_generator_))
    return ControlSignalType::JointVelocities;
  else if (std::holds_alternative<MotionGenerator<franka::JointPositions>>(motion_generator_))
    return ControlSignalType::JointPositions;
  else if (std::holds_alternative<MotionGenerator<franka::CartesianVelocities>>(motion_generator_))
    return ControlSignalType::CartesianVelocities;
  else
    return ControlSignalType::CartesianPose;
}

RelativeDynamicsFactor Robot::relative_dynamics_factor() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  return params_.relative_dynamics_factor;
}

void Robot::setRelativeDynamicsFactor(const RelativeDynamicsFactor &relative_dynamics_factor) {
  std::unique_lock<std::mutex> lock(control_mutex_);
  params_.relative_dynamics_factor = relative_dynamics_factor;
}

}  // namespace franky
