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
  if ((waypoint.target.vector_repr().head<6>().array().abs() > vel_lim.head<6>().array()).any()) {
    std::stringstream ss;
    ss << "Waypoint velocity " << waypoint.target.vector_repr().head<6>() << " exceeds maximum velocity "
       << vel_lim.head<6>() << ".";
    throw std::runtime_error(ss.str());
  }
}

void CartesianVelocityWaypointMotion::initWaypointMotion(
    const franka::RobotState &robot_state,
    const std::optional<franka::CartesianVelocities> &previous_command,
    ruckig::InputParameter<7> &input_parameter) {
  RobotVelocity current_velocity
      (previous_command.value_or(franka::CartesianVelocities{robot_state.O_dP_EE_c, robot_state.delbow_c}));

  auto initial_acceleration = Vector6d::Map(robot_state.O_ddP_EE_c.data());
  Vector7d initial_acceleration_with_elbow = (Vector7d() << initial_acceleration, robot_state.ddelbow_c[0]).finished();

  input_parameter.current_position = toStd<7>(current_velocity.vector_repr());
  input_parameter.current_velocity = toStd<7>(initial_acceleration_with_elbow);
  input_parameter.current_acceleration = toStd<7>(Vector7d::Zero());

  if (previous_command.has_value())
    last_elbow_pos_ = previous_command.value().elbow[0];
  else
    last_elbow_pos_ = robot_state.elbow_c[0];
  last_elbow_vel_ = robot_state.delbow_c[0];
}

franka::CartesianVelocities CartesianVelocityWaypointMotion::getControlSignal(
    const franka::Duration &time_step, const ruckig::InputParameter<7> &input_parameter) {
  auto has_elbow = input_parameter.enabled[6];
  RobotVelocity target_vel(toEigenD<7>(input_parameter.current_position));
  if (has_elbow) {
    auto time_step_s = time_step.toSec();
    auto current_elbow_vel = input_parameter.current_position[6];
    auto current_elbow_acc = input_parameter.current_velocity[6];
    auto last_elbow_acc = 2 * (current_elbow_vel - last_elbow_vel_) / time_step_s - current_elbow_acc;
    auto elbow_jerk = (current_elbow_acc - last_elbow_acc) / time_step_s;
    auto current_elbow_pos =
        last_elbow_pos_
            + current_elbow_vel * time_step_s
            + 0.5 * current_elbow_acc * std::pow(time_step_s, 2)
            + 1.0 / 6.0 * elbow_jerk * std::pow(time_step_s, 3);

    last_elbow_vel_ = input_parameter.current_position[0];
    last_elbow_pos_ = current_elbow_pos;
    return  target_vel.as_franka_velocity(current_elbow_pos);
  }
  return target_vel.as_franka_velocity();
}

void CartesianVelocityWaypointMotion::setNewWaypoint(
    const franka::RobotState &robot_state,
    const std::optional<franka::CartesianVelocities> &previous_command,
    const VelocityWaypoint<RobotVelocity> &new_waypoint,
    ruckig::InputParameter<7> &input_parameter) {
  auto new_target_transformed = new_waypoint.target.changeEndEffectorFrame(ee_frame_.inverse().translation());
  // This is a bit of an oversimplification, as the angular velocities don't work like linear velocities (but we pretend
  // they do here). However, it is probably good enough here.
  input_parameter.target_position = toStd<7>(new_target_transformed.vector_repr());
  input_parameter.target_velocity = toStd<7>(Vector7d::Zero());
  input_parameter.enabled = {true, true, true, true, true, true, new_target_transformed.elbow_velocity().has_value()};
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

}  // namespace franky
