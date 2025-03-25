#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"
#include "franky/motion/motion.hpp"
#include "franky/relative_dynamics_factor.hpp"
#include "franky/motion/reference_type.hpp"

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
 */
template<typename TargetType>
struct Waypoint {
  TargetType target;

  ReferenceType reference_type{ReferenceType::Absolute};

  RelativeDynamicsFactor relative_dynamics_factor{1.0};

  std::optional<double> minimum_time{std::nullopt};
};

/**
 * @brief A motion following multiple waypoints in a time-optimal way. Works with arbitrary initial conditions.
 * @tparam ControlSignalType The type of the control signal. Either franka::Torques, franka::JointVelocities,
 * franka::CartesianVelocities, franka::JointPositions or franka::CartesianPose.
 * @tparam TargetType The type of the target of the waypoints.
 */
template<typename ControlSignalType, typename TargetType>
class WaypointMotion : public Motion<ControlSignalType> {
 public:
  /**
   * @brief Parameters for the waypoint motion.
   */
  struct Params {
    /**
     * The relative dynamics factor for this motion. This factor will get multiplied with the robot's global dynamics
     * factor to get the actual dynamics factor for this motion.
     */
    RelativeDynamicsFactor relative_dynamics_factor{1.0};

    /**
     * Whether to end the motion when the last waypoint is reached or keep holding the last target.
     */
    bool return_when_finished{true};
  };

  /**
   * @param waypoints The waypoints to follow.
   * @param params Parameters for the motion.
   */
  explicit WaypointMotion(std::vector<Waypoint<TargetType>> waypoints, Params params)
      : waypoints_(std::move(waypoints)), params_(std::move(params)), prev_result_() {}

 protected:
  void initImpl(const franka::RobotState &robot_state,
                const std::optional<ControlSignalType> &previous_command) override {

    initWaypointMotion(robot_state, previous_command, input_para_);

    waypoint_iterator_ = waypoints_.begin();
    if (waypoint_iterator_ != waypoints_.end()) {
      setNewWaypoint(robot_state, previous_command, *waypoint_iterator_, input_para_);
      setInputLimits(*waypoint_iterator_);
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
        if (waypoint_iterator_ != waypoints_.end())
          ++waypoint_iterator_;
        if (waypoint_iterator_ == waypoints_.end()) {
          auto output_pose = getControlSignal(input_para_);
          if (!params_.return_when_finished)
            return output_pose;
          return franka::MotionFinished(output_pose);
        } else {
          setNewWaypoint(robot_state, previous_command, *waypoint_iterator_, input_para_);
          setInputLimits(*waypoint_iterator_);
        }
      }
      if (waypoint_iterator_ != waypoints_.end()) {
        prev_result_ = trajectory_generator_.update(input_para_, output_para_);
        if (prev_result_ == ruckig::Result::Error) {
          throw MotionPlannerException("Invalid inputs to motion planner.");
        }
        output_para_.pass_to_input(input_para_);
      }
    }

    return getControlSignal(input_para_);
  };

  virtual void initWaypointMotion(
      const franka::RobotState &robot_state,
      const std::optional<ControlSignalType> &previous_command,
      ruckig::InputParameter<7> &input_parameter) = 0;

  virtual void setNewWaypoint(
      const franka::RobotState &robot_state,
      const std::optional<ControlSignalType> &previous_command,
      const Waypoint<TargetType> &new_waypoint,
      ruckig::InputParameter<7> &input_parameter) = 0;

  [[nodiscard]] virtual std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const = 0;

  [[nodiscard]] virtual ControlSignalType getControlSignal(const ruckig::InputParameter<7> &input_parameter) const = 0;

 private:
  std::vector<Waypoint<TargetType>> waypoints_;
  Params params_;

  ruckig::Ruckig<7> trajectory_generator_{Robot::control_rate};
  ruckig::Result prev_result_;
  ruckig::InputParameter<7> input_para_;
  ruckig::OutputParameter<7> output_para_;

  typename std::vector<Waypoint<TargetType>>::iterator waypoint_iterator_;

  void setInputLimits(const Waypoint<TargetType> &waypoint) {
    auto robot = this->robot();

    auto [vel_lim, acc_lim, jerk_lim] = getAbsoluteInputLimits();

    auto relative_dynamics_factor =
        waypoint.relative_dynamics_factor * params_.relative_dynamics_factor * robot->relative_dynamics_factor();

    input_para_.max_velocity = toStd<7>(relative_dynamics_factor.velocity() * vel_lim);
    input_para_.max_acceleration = toStd<7>(relative_dynamics_factor.acceleration() * acc_lim);
    input_para_.max_jerk = toStd<7>(relative_dynamics_factor.jerk() * jerk_lim);

    if (relative_dynamics_factor.max_dynamics()) {
      input_para_.synchronization = ruckig::Synchronization::TimeIfNecessary;
    } else {
      input_para_.synchronization = ruckig::Synchronization::Time;
      if (waypoint.minimum_time.has_value())
        input_para_.minimum_duration = waypoint.minimum_time.value();
    }
  }
};

}  // namespace franky
