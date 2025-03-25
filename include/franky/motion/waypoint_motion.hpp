#pragma once

#include <optional>
#include <ruckig/ruckig.hpp>

#include "franky/motion/motion.hpp"

namespace franky {

/**
 * @brief Exception thrown if the motion planner fails.
 */
struct MotionPlannerException : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

/**
 * @brief A waypoint with a target and optional parameters.
 *
 * @tparam TargetType The type of the target.
 *
 * @param target The target of this waypoint.
 * @param reference_type The reference type (absolute or relative).
 * @param relative_dynamics_factor The relative dynamics factor for this waypoint. This factor will get multiplied with
 * the robot's global dynamics factor and the motion dynamics factor to get the actual dynamics factor for this
 * waypoint.
 * @param minimum_time The minimum time to get to the next waypoint in [s].
 * @param hold_target_duration For how long to hold the target of this waypoint after it has been reached.
 */
template<typename TargetType>
struct Waypoint {
  TargetType target;

  RelativeDynamicsFactor relative_dynamics_factor{1.0};

  std::optional<double> minimum_time{std::nullopt};

  std::chrono::duration<double> hold_target_duration{0.0};
};

/**
 * @brief A motion following multiple waypoints in a time-optimal way. Works with arbitrary initial conditions.
 * @tparam ControlSignalType    The type of the control signal. Either franka::Torques, franka::JointVelocities,
 *                              franka::CartesianVelocities, franka::JointPositions or franka::CartesianPose.
 * @tparam WaypointType         The type of the waypoints. Must subclass Waypoint<TargetType>.
 * @tparam TargetType           The type of the target of the waypoints.
 */
template<typename ControlSignalType, typename WaypointType, typename TargetType>
class WaypointMotion : public Motion<ControlSignalType> {
  static_assert(std::is_base_of_v<Waypoint<TargetType>, WaypointType>,
                "WaypointType must inherit from Waypoint<TargetType>");
 public:
  /**
   * @param waypoints                The waypoints to follow.
   * @param return_when_finished     Whether to end the motion when the last waypoint is reached or keep holding the
   *                                 last target.
   */
  explicit WaypointMotion(std::vector<WaypointType> waypoints, bool return_when_finished = true)
      : waypoints_(std::move(waypoints)), return_when_finished_(return_when_finished), prev_result_() {}

 protected:
  void initImpl(const franka::RobotState &robot_state,
                const std::optional<ControlSignalType> &previous_command) override {
    target_reached_time_ = std::nullopt;

    initWaypointMotion(robot_state, previous_command, input_parameter_);

    waypoint_iterator_ = waypoints_.begin();
    if (waypoint_iterator_ != waypoints_.end()) {
      setNewWaypoint(robot_state, previous_command, *waypoint_iterator_, input_parameter_);
      setInputLimits(*waypoint_iterator_, input_parameter_);
      prev_result_ = ruckig::Result::Working;
    } else {
      prev_result_ = ruckig::Result::Finished;
    }
  }

  ControlSignalType
  nextCommandImpl(
      const franka::RobotState &robot_state,
      franka::Duration time_step,
      std::chrono::duration<double> rel_time,
      std::chrono::duration<double> abs_time,
      const std::optional<ControlSignalType> &previous_command) override {
    const uint64_t steps = std::max<uint64_t>(time_step.toMSec(), 1);
    for (size_t i = 0; i < steps; i++) {
      if (prev_result_ == ruckig::Result::Finished) {
        if (!target_reached_time_.has_value()) {
          target_reached_time_ = rel_time;
        }
        if (rel_time - target_reached_time_.value() >= waypoint_iterator_->hold_target_duration) {
          target_reached_time_ = std::nullopt;
          if (waypoint_iterator_ != waypoints_.end()) {
            ++waypoint_iterator_;
          }
        }
        if (waypoint_iterator_ == waypoints_.end()) {
          auto output_pose = getControlSignal(input_parameter_);
          if (!return_when_finished_)
            return output_pose;
          return franka::MotionFinished(output_pose);
        } else {
          setNewWaypoint(robot_state, previous_command, *waypoint_iterator_, input_parameter_);
          setInputLimits(*waypoint_iterator_, input_parameter_);
        }
      }
      if (waypoint_iterator_ != waypoints_.end()) {
        prev_result_ = trajectory_generator_.update(input_parameter_, output_parameter_);
        if (prev_result_ == ruckig::Result::Error) {
          throw MotionPlannerException("Invalid inputs to motion planner.");
        }
        output_parameter_.pass_to_input(input_parameter_);
      }
    }

    return getControlSignal(input_parameter_);
  };

  virtual void initWaypointMotion(
      const franka::RobotState &robot_state,
      const std::optional<ControlSignalType> &previous_command,
      ruckig::InputParameter<7> &input_parameter) = 0;

  virtual void setNewWaypoint(
      const franka::RobotState &robot_state,
      const std::optional<ControlSignalType> &previous_command,
      const WaypointType &new_waypoint,
      ruckig::InputParameter<7> &input_parameter) = 0;

  [[nodiscard]] virtual std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const = 0;

  [[nodiscard]] virtual ControlSignalType getControlSignal(const ruckig::InputParameter<7> &input_parameter) const = 0;

  virtual void setInputLimits(const WaypointType &waypoint, ruckig::InputParameter<7> &input_parameter) const = 0;

 private:
  std::vector<WaypointType> waypoints_;
  bool return_when_finished_{true};

  ruckig::Ruckig<7> trajectory_generator_{Robot::control_rate};
  ruckig::Result prev_result_;
  ruckig::InputParameter<7> input_parameter_;
  ruckig::OutputParameter<7> output_parameter_;

  typename std::vector<WaypointType>::iterator waypoint_iterator_;

  std::optional<std::chrono::duration<double>> target_reached_time_;

};

}  // namespace franky
