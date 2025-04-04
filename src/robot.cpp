#include "franky/robot.hpp"

#include <franky/rt_mutex.hpp>

#include "franky/types.hpp"
#include "franky/util.hpp"

namespace franky {

//! Connects to a robot at the given FCI IP address.
Robot::Robot(const std::string &fci_hostname) : Robot(fci_hostname, Params()) {}

Robot::Robot(const std::string &fci_hostname, const Params &params)
    : fci_hostname_(fci_hostname),
      params_(params),
      control_mutex_(std::make_shared<std::mutex>()),
      franka::Robot(fci_hostname, params.realtime_config) {
  patchMutexRT(state_mutex_);
  patchMutexRT(*control_mutex_);
  model_ = std::make_shared<const Model>(loadModel());
#ifdef FRANKA_0_15
  model_urdf_ = getRobotModel();
#endif
  setCollisionBehavior(params_.default_torque_threshold, params_.default_force_threshold);
}

bool Robot::hasErrors() { return static_cast<bool>(state().current_errors); }

bool Robot::recoverFromErrors() {
  automaticErrorRecovery();
  return !hasErrors();
}

RobotState Robot::state() {
  std::lock_guard state_lock(state_mutex_);
  {
    std::lock_guard control_lock(*control_mutex_);
    if (!is_in_control_unsafe()) {
      current_state_ = RobotState::from_franka(readOnce());
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
  std::unique_lock lock(*control_mutex_);
  return is_in_control_unsafe();
}

std::string Robot::fci_hostname() const {
  return fci_hostname_;
}

std::optional<ControlSignalType> Robot::current_control_signal_type() {
  std::unique_lock lock(*control_mutex_);
  if (!is_in_control_unsafe())
    return std::nullopt;
  if (std::holds_alternative<MotionGenerator<franka::Torques>>(motion_generator_))
    return Torques;
  if (std::holds_alternative<MotionGenerator<franka::JointVelocities>>(motion_generator_))
    return JointVelocities;
  if (std::holds_alternative<MotionGenerator<franka::JointPositions>>(motion_generator_))
    return JointPositions;
  if (std::holds_alternative<MotionGenerator<franka::CartesianVelocities>>(motion_generator_))
    return CartesianVelocities;
  return CartesianPose;
}

RelativeDynamicsFactor Robot::relative_dynamics_factor() {
  std::unique_lock lock(*control_mutex_);
  return params_.relative_dynamics_factor;
}

void Robot::setRelativeDynamicsFactor(const RelativeDynamicsFactor &relative_dynamics_factor) {
  std::unique_lock lock(*control_mutex_);
  params_.relative_dynamics_factor = relative_dynamics_factor;
}

}  // namespace franky
