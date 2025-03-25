#include "franky/motion/joint_velocity_waypoint_motion.hpp"

#include <ruckig/ruckig.hpp>

namespace franky {

JointVelocityWaypointMotion::JointVelocityWaypointMotion(
    const std::vector<VelocityWaypoint<Vector7d>> &waypoints,
    const RelativeDynamicsFactor &relative_dynamics_factor)
    : VelocityWaypointMotion<franka::JointVelocities, Vector7d>(waypoints, relative_dynamics_factor) {}

void JointVelocityWaypointMotion::checkWaypoint(const VelocityWaypoint<Vector7d> &waypoint) const {
  auto [vel_lim, acc_lim, jerk_lim] = getAbsoluteInputLimits();
  if ((waypoint.target.array().abs() > vel_lim.array()).any()) {
    std::stringstream ss;
    ss << "Waypoint velocity " << waypoint.target << " exceeds maximum velocity " << vel_lim << ".";
    throw std::runtime_error(ss.str());
  }
}

void JointVelocityWaypointMotion::initWaypointMotion(
    const franka::RobotState &robot_state,
    const std::optional<franka::JointVelocities> &previous_command,
    ruckig::InputParameter<7> &input_parameter) {
  if (previous_command.has_value())
    input_parameter.current_position = previous_command->dq;
  else
    input_parameter.current_position = robot_state.dq_d;
  input_parameter.current_velocity = robot_state.ddq_d;
  input_parameter.current_acceleration = toStd<7>(Vector7d::Zero());
}

franka::JointVelocities JointVelocityWaypointMotion::getControlSignal(
    const ruckig::InputParameter<7> &input_parameter) const {
  return {input_parameter.current_position};
}

void JointVelocityWaypointMotion::setNewWaypoint(
    const franka::RobotState &robot_state,
    const std::optional<franka::JointVelocities> &previous_command,
    const VelocityWaypoint<Vector7d> &new_waypoint,
    ruckig::InputParameter<7> &input_parameter) {
  auto new_target = new_waypoint.target;
  input_parameter.target_position = toStd<7>(new_target);
  input_parameter.target_velocity = toStd<7>(Vector7d::Zero());
  input_parameter.target_acceleration = toStd<7>(Vector7d::Zero());
}

std::tuple<Vector7d, Vector7d, Vector7d> JointVelocityWaypointMotion::getAbsoluteInputLimits() const {
  return {
      Vector7d::Map(Robot::max_joint_velocity.data()),
      Vector7d::Map(Robot::max_joint_acceleration.data()),
      Vector7d::Map(Robot::max_joint_jerk.data())
  };
}
}  // namespace franky
