#include "franky/motion/cartesian_velocity_waypoint_motion.hpp"

#include <ruckig/ruckig.hpp>
#include <utility>

#include "franky/cartesian_state.hpp"
#include "franky/robot.hpp"
#include "franky/util.hpp"

namespace franky {

CartesianVelocityWaypointMotion::CartesianVelocityWaypointMotion(
    const std::vector<VelocityWaypoint<RobotVelocity>> &waypoints,
    const RelativeDynamicsFactor &relative_dynamics_factor,
    Affine ee_frame)
    : VelocityWaypointMotion<franka::CartesianVelocities, RobotVelocity>(waypoints, relative_dynamics_factor),
      ee_frame_(std::move(ee_frame)) {}

void CartesianVelocityWaypointMotion::checkWaypoint(const VelocityWaypoint<RobotVelocity> &waypoint) const {
  auto [vel_lim, acc_lim, jerk_lim] = getAbsoluteInputLimits();
  if ((waypoint.target.vector_repr().array().abs() > vel_lim.array()).any()) {
    std::stringstream ss;
    ss << "Waypoint velocity " << waypoint.target.vector_repr() << " exceeds maximum velocity " << vel_lim << ".";
    throw std::runtime_error(ss.str());
  }
}

void CartesianVelocityWaypointMotion::initWaypointMotion(
    const franka::RobotState &robot_state,
    const std::optional<franka::CartesianVelocities> &previous_command,
    ruckig::InputParameter<7> &input_parameter) {
  RobotVelocity current_velocity
      (previous_command.value_or(franka::CartesianVelocities{robot_state.O_dP_EE_c, robot_state.delbow_c}));

  // The elbow treatment is not a bug. libfranka expects elbow positions here and not elbow velocities!
  auto initial_acceleration = Vector6d::Map(robot_state.O_ddP_EE_c.data());
  Vector7d initial_acceleration_with_elbow = (Vector7d() << initial_acceleration, robot_state.delbow_c[0]).finished();

  target_state_ = current_velocity;

  Vector7d jerk_component;
  jerk_component << Vector6d::Zero(), robot_state.ddelbow_c[0];
  input_parameter.current_position = toStd<7>(current_velocity.withElbow(robot_state.elbow[0]).vector_repr());
  input_parameter.current_velocity = toStd<7>(initial_acceleration_with_elbow);
  input_parameter.current_acceleration = toStd<7>(jerk_component);
}

franka::CartesianVelocities CartesianVelocityWaypointMotion::getControlSignal(
    const ruckig::InputParameter<7> &input_parameter) const {
  auto has_elbow = input_parameter.enabled[6];
  return RobotVelocity(toEigen<7>(input_parameter.current_position), !has_elbow).as_franka_velocity();
}

void CartesianVelocityWaypointMotion::setNewWaypoint(
    const franka::RobotState &robot_state,
    const std::optional<franka::CartesianVelocities> &previous_command,
    const VelocityWaypoint<RobotVelocity> &new_waypoint,
    ruckig::InputParameter<7> &input_parameter) {
  auto new_target_transformed = new_waypoint.target.changeEndEffectorFrame(ee_frame_.inverse().translation());
  // This is a bit of an oversimplification, as the angular velocities don't work like linear velocities (but we pretend
  // they do here). However, it is probably good enough here.
  input_parameter.target_position = toStd<7>(new_waypoint.target.vector_repr());
  input_parameter.target_velocity = toStd<7>(Vector7d::Zero());
  input_parameter.enabled = {true, true, true, true, true, true, new_target_transformed.elbow().has_value()};
  target_state_ = new_waypoint.target;
}

std::tuple<Vector7d, Vector7d, Vector7d>
CartesianVelocityWaypointMotion::getAbsoluteInputLimits() const {
  Vector7d max_vel = vec_cart_rot_elbow(
      Robot::max_translation_velocity, Robot::max_rotation_velocity, Robot::max_elbow_velocity);
  Vector7d max_acc = vec_cart_rot_elbow(
      Robot::max_translation_acceleration, Robot::max_rotation_acceleration, Robot::max_elbow_acceleration);
  Vector7d max_jerk = vec_cart_rot_elbow(
      Robot::max_translation_jerk, Robot::max_rotation_jerk, Robot::max_elbow_jerk);
  return {max_vel, max_acc, max_jerk};
}

void CartesianVelocityWaypointMotion::setInputLimits(
    const VelocityWaypoint<RobotVelocity> &waypoint, ruckig::InputParameter<7> &input_parameter) const {
  // Have to rewrite this code here to deal with the elbow
  auto [vel_lim, acc_lim, jerk_lim] = getAbsoluteInputLimits();

  auto relative_dynamics_factor =
      waypoint.relative_dynamics_factor * relative_dynamics_factor_ * this->robot()->relative_dynamics_factor();

  Vector7d vel_component;
  vel_component << relative_dynamics_factor.acceleration() * acc_lim.head<6>(),
      relative_dynamics_factor.velocity() * vel_lim[6];

  Vector7d acc_component;
  acc_component << relative_dynamics_factor.jerk() * jerk_lim.head<6>(),
      relative_dynamics_factor.acceleration() * acc_lim[6];

  Vector7d jerk_component;
  jerk_component << Vector6d::Constant(std::numeric_limits<double>::infinity()),
  relative_dynamics_factor.jerk() * jerk_lim[6];

  input_parameter.max_velocity = toStd<7>(vel_component);
  input_parameter.max_acceleration = toStd<7>(acc_component);
  input_parameter.max_jerk = toStd<7>(jerk_component);

  if (relative_dynamics_factor.max_dynamics()) {
    input_parameter.synchronization = ruckig::Synchronization::TimeIfNecessary;
  } else {
    input_parameter.synchronization = ruckig::Synchronization::Time;
    if (waypoint.minimum_time.has_value())
      input_parameter.minimum_duration = waypoint.minimum_time.value();
  }
}

}  // namespace franky
