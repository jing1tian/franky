#include "franky/motion/joint_waypoint_motion.hpp"

#include <ruckig/ruckig.hpp>

#include "franky/motion/reference_type.hpp"

namespace franky {

JointWaypointMotion::JointWaypointMotion(
    const std::vector<PositionWaypoint<JointState>> &waypoints,
    const RelativeDynamicsFactor &relative_dynamics_factor,
    bool return_when_finished)
    : PositionWaypointMotion<franka::JointPositions, JointState>(
        waypoints, relative_dynamics_factor, return_when_finished) {}

void JointWaypointMotion::initWaypointMotion(
    const franka::RobotState &robot_state,
    const std::optional<franka::JointPositions> &previous_command,
    ruckig::InputParameter<7> &input_parameter) {
  if (previous_command.has_value())
    input_parameter.current_position = previous_command->q;
  else
    input_parameter.current_position = robot_state.q_d;
  input_parameter.current_velocity = robot_state.dq_d;
  input_parameter.current_acceleration = robot_state.ddq_d;
}

franka::JointPositions JointWaypointMotion::getControlSignal(
    const franka::Duration &time_step, const ruckig::InputParameter<7> &input_parameter) {
  return {input_parameter.current_position};
}

void JointWaypointMotion::setNewWaypoint(
    const franka::RobotState &robot_state,
    const std::optional<franka::JointPositions> &previous_command,
    const PositionWaypoint<JointState> &new_waypoint,
    ruckig::InputParameter<7> &input_parameter) {
  auto new_target = new_waypoint.target;
  if (new_waypoint.reference_type == ReferenceType::Relative)
    new_target.position() += toEigen(input_parameter.current_position);
  input_parameter.target_position = toStd<7>(new_target.position());
  input_parameter.target_velocity = toStd<7>(new_target.velocity());
  input_parameter.target_acceleration = toStd<7>(Vector7d::Zero());
}

std::tuple<Vector7d, Vector7d, Vector7d> JointWaypointMotion::getAbsoluteInputLimits() const {
  return {
      Vector7d::Map(Robot::max_joint_velocity.data()),
      Vector7d::Map(Robot::max_joint_acceleration.data()),
      Vector7d::Map(Robot::max_joint_jerk.data())
  };
}
}  // namespace franky
