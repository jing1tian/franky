#pragma once

#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"
#include "franky/relative_dynamics_factor.hpp"
#include "franky/motion/reference_type.hpp"
#include "franky/motion/waypoint_motion.hpp"

namespace franky {

/**
 * @brief A position waypoint with a target and optional parameters.
 *
 * @tparam TargetType The type of the target.
 *
 * @param reference_type The reference type (absolute or relative).
 */
template<typename TargetType>
struct PositionWaypoint : public Waypoint<TargetType> {
  ReferenceType reference_type{ReferenceType::Absolute};
};

/**
 * @brief A motion following multiple positional waypoints in a time-optimal way. Works with arbitrary initial
 * conditions.
 * @tparam ControlSignalType The type of the control signal. Either franka::Torques, franka::JointVelocities,
 * franka::CartesianVelocities, franka::JointPositions or franka::CartesianPose.
 * @tparam TargetType The type of the target of the waypoints.
 */
template<typename ControlSignalType, typename TargetType>
class PositionWaypointMotion : public WaypointMotion<ControlSignalType, PositionWaypoint<TargetType>, TargetType> {
 public:
  /**
   * @param waypoints                The waypoints to follow.
   * @param relative_dynamics_factor The relative dynamics factor for this motion. This factor will get multiplied with
   *                                 the robot's global dynamics factor to get the actual dynamics factor for this
   *                                 motion.
   * @param return_when_finished     Whether to end the motion when the last waypoint is reached or keep holding the
   *                                 last target.
   */
  explicit PositionWaypointMotion(
      std::vector<PositionWaypoint<TargetType>> waypoints,
      const RelativeDynamicsFactor &relative_dynamics_factor = 1.0,
      bool return_when_finished = true
  )
      : WaypointMotion<ControlSignalType, PositionWaypoint<TargetType>, TargetType>(waypoints, return_when_finished),
        relative_dynamics_factor_(relative_dynamics_factor) {}

 protected:
  void setInputLimits(
      const PositionWaypoint<TargetType> &waypoint, ruckig::InputParameter<7> &input_parameter) const override {
    auto [vel_lim, acc_lim, jerk_lim] = getAbsoluteInputLimits();

    auto relative_dynamics_factor =
        waypoint.relative_dynamics_factor * relative_dynamics_factor_ * this->robot()->relative_dynamics_factor();

    input_parameter.max_velocity = toStd<7>(relative_dynamics_factor.velocity() * vel_lim);
    input_parameter.max_acceleration = toStd<7>(relative_dynamics_factor.acceleration() * acc_lim);
    input_parameter.max_jerk = toStd<7>(relative_dynamics_factor.jerk() * jerk_lim);

    if (relative_dynamics_factor.max_dynamics()) {
      input_parameter.synchronization = ruckig::Synchronization::TimeIfNecessary;
    } else {
      input_parameter.synchronization = ruckig::Synchronization::Time;
      if (waypoint.minimum_time.has_value())
        input_parameter.minimum_duration = waypoint.minimum_time.value();
    }
  }

  [[nodiscard]] std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override = 0;

 private:
  RelativeDynamicsFactor relative_dynamics_factor_;
};

}  // namespace franky
