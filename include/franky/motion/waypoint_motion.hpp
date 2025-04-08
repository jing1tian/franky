#pragma once

#include <optional>
#include <ruckig/ruckig.hpp>

#include "franky/motion/motion.hpp"

namespace franky {

/**
 * @brief Exception thrown if the motion planner fails.
 */
struct MotionPlannerException : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/**
 * @brief A waypoint with a target and optional parameters.
 *
 * @tparam TargetType The type of the target.
 *
 * @param target                    The target of this waypoint.
 * @param reference_type            The reference type (absolute or relative).
 * @param relative_dynamics_factor  The relative dynamics factor for this waypoint. This factor will get multiplied with
 *                                  the robot's global dynamics factor and the motion dynamics factor to get the actual
 *                                  dynamics factor for this waypoint.
 * @param minimum_time              The minimum time to get to the next waypoint.
 * @param hold_target_duration      For how long to hold the target of this waypoint after it has been reached.
 * @param max_total_duration        The maximum time to try reaching this waypoint before moving on to the next
 *                                  waypoint. Default is infinite.
 * @param state_estimate_weight     Weighting of the robot state estimate vs the target when computing the current state
 *                                  to continue planning from. A value of 0 means that the planner always assumes it
 *                                  reached its last target perfectly (open loop control), while a value of 1 means that
 *                                  the planner always uses the robot state estimate (closed loop control). A value
 *                                  between 0 and 1 means that the planner uses a weighted average of the two.
 */
template <typename TargetType>
struct Waypoint {
  TargetType target;

  RelativeDynamicsFactor relative_dynamics_factor{1.0};

  std::optional<franka::Duration> minimum_time{std::nullopt};

  franka::Duration hold_target_duration{0};

  std::optional<franka::Duration> max_total_duration{std::nullopt};

  Eigen::Vector3d state_estimate_weight{0.0, 0.0, 0.0};
};

/**
 * @brief A motion following multiple waypoints in a time-optimal way. Works with arbitrary initial conditions.
 * @tparam ControlSignalType    The type of the control signal. Either franka::Torques, franka::JointVelocities,
 *                              franka::CartesianVelocities, franka::JointPositions or franka::CartesianPose.
 * @tparam WaypointType         The type of the waypoints. Must subclass Waypoint<TargetType>.
 * @tparam TargetType           The type of the target of the waypoints.
 */
template <typename ControlSignalType, typename WaypointType, typename TargetType>
class WaypointMotion : public Motion<ControlSignalType> {
  static_assert(
      std::is_base_of_v<Waypoint<TargetType>, WaypointType>, "WaypointType must inherit from Waypoint<TargetType>");

 public:
  /**
   * @param waypoints                The waypoints to follow.
   * @param return_when_finished     Whether to end the motion when the last waypoint is reached or keep holding the
   *                                 last target.
   */
  explicit WaypointMotion(std::vector<WaypointType> waypoints, bool return_when_finished = true)
      : waypoints_(std::move(waypoints)), return_when_finished_(return_when_finished), prev_result_() {}

 protected:
  void initImpl(const RobotState &robot_state, const std::optional<ControlSignalType> &previous_command) override {
    target_reached_time_ = std::nullopt;
    waypoint_started_time_ = franka::Duration(0);

    initWaypointMotion(robot_state, previous_command, input_parameter_);

    waypoint_iterator_ = waypoints_.begin();
    prev_close_to_target_ = false;
    first_update_ = true;
  }

  ControlSignalType nextCommandImpl(
      const RobotState &robot_state, franka::Duration time_step, franka::Duration rel_time, franka::Duration abs_time,
      const std::optional<ControlSignalType> &previous_command) override {
    if (first_update_) {
      // We do this twice on purpose, as the first time is just to ensure that input_parameter_ is initialized for the
      // state estimator warm-up
      initWaypointMotion(robot_state, previous_command, input_parameter_);
      if (waypoint_iterator_ != waypoints_.end()) {
        checkWaypointPrivate(*waypoint_iterator_);
        setNewWaypoint(robot_state, previous_command, *waypoint_iterator_, input_parameter_);
        setInputLimits(*waypoint_iterator_, input_parameter_);
        prev_result_ = ruckig::Result::Working;
      } else {
        prev_result_ = ruckig::Result::Finished;
      }
      first_update_ = false;
    }

    const auto expected_time_step = franka::Duration(1);
    if (time_step > expected_time_step) {
      // In this case, we missed a couple of steps for some reason. Hence, extrapolate the way the robot does if it
      // does not receive data (constant acceleration model).
      // See https://frankaemika.github.io/docs/libfranka.html#under-the-hood
      extrapolateMotion(robot_state, time_step - expected_time_step, input_parameter_, output_parameter_);
      output_parameter_.pass_to_input(input_parameter_);
    }

    auto max_time_reached = false;
    if (waypoint_iterator_ != waypoints_.end() && waypoint_iterator_->max_total_duration.has_value()) {
      max_time_reached = rel_time - waypoint_started_time_ >= waypoint_iterator_->max_total_duration.value();
    }

    if (prev_result_ == ruckig::Finished || max_time_reached) {
      if (!target_reached_time_.has_value()) {
        target_reached_time_ = rel_time;
      }
      if (waypoint_iterator_ != waypoints_.end()) {
        auto hold_target_duration = waypoint_iterator_->hold_target_duration;
        if (waypoint_iterator_ + 1 == waypoints_.end()) {
          // Allow cooldown of motion, so that the low-pass filter has time to adjust to target values
          hold_target_duration = std::max(hold_target_duration, franka::Duration(10));
        }
        if (rel_time - target_reached_time_.value() >= hold_target_duration || max_time_reached) {
          target_reached_time_ = std::nullopt;
          ++waypoint_iterator_;
          if (waypoint_iterator_ != waypoints_.end()) {
            checkWaypointPrivate(*waypoint_iterator_);
            setNewWaypoint(robot_state, previous_command, *waypoint_iterator_, input_parameter_);
            setInputLimits(*waypoint_iterator_, input_parameter_);
            waypoint_started_time_ = rel_time;
          }
        }
      }
      if (waypoint_iterator_ == waypoints_.end()) {
        auto command = getControlSignal(robot_state, time_step, previous_command, input_parameter_);
        if (!return_when_finished_) return command;
        return franka::MotionFinished(command);
      }
    }

    assert(waypoint_iterator_ != waypoints_.end());

    auto [curr_pos, curr_vel, curr_acc] = getStateEstimate(robot_state);

    const auto tar_pos = toEigenD(input_parameter_.target_position);
    const auto tar_vel = toEigenD(input_parameter_.target_velocity);
    const auto tar_acc = toEigenD(input_parameter_.target_acceleration);

    const auto pos_error = (curr_pos - tar_pos).cwiseAbs();
    const auto vel_error = (curr_vel - tar_vel).cwiseAbs();
    const auto acc_error = (curr_acc - tar_acc).cwiseAbs();

    const auto [prec_pos, prec_vel, prec_acc] = getGoalCloseTolerance();

    const auto close_to_target = (pos_error - prec_pos).maxCoeff() < 0 && (vel_error - prec_vel).maxCoeff() < 0 &&
                                 (acc_error - prec_acc).maxCoeff() < 0;

    auto pos_t = toEigen(input_parameter_.current_position);
    auto vel_t = toEigen(input_parameter_.current_velocity);
    auto acc_t = toEigen(input_parameter_.current_acceleration);

    auto [curr_pos_d, curr_vel_d, curr_acc_d] = getDesiredState(robot_state);
    Eigen::Vector3d blend = waypoint_iterator_->state_estimate_weight;
    if (close_to_target && blend.maxCoeff() > 0) {
      // Switch to open-loop control close to the target to avoid precision problems
      if (!prev_close_to_target_) {
        pos_t = curr_pos_d;
        vel_t = curr_vel_d;
        acc_t = curr_acc_d;
      }
      blend = Eigen::Vector3d::Zero();
    }
    prev_close_to_target_ = close_to_target;

    auto pos_b = (1 - blend[0]) * pos_t + blend[0] * curr_pos;
    auto vel_b = (1 - blend[1]) * vel_t + blend[1] * curr_vel;
    auto acc_b = (1 - blend[2]) * acc_t + blend[2] * curr_acc;

    input_parameter_.current_position = toStdD<7>(pos_b);
    input_parameter_.current_velocity = toStdD<7>(vel_b);
    input_parameter_.current_acceleration = toStdD<7>(acc_b);

    prev_result_ = trajectory_generator_.update(input_parameter_, output_parameter_);
    if (prev_result_ == ruckig::Result::Working || prev_result_ == ruckig::Result::Finished) {
      // This is a workaround to prevent NaNs from popping up. Must be some bug in ruckig.
      for (int i = 0; i < 7; i++) {
        if (!input_parameter_.enabled[i]) {
          output_parameter_.new_position[i] = 0.0;
          output_parameter_.new_velocity[i] = 0.0;
          output_parameter_.new_acceleration[i] = 0.0;
        }
      }
      output_parameter_.pass_to_input(input_parameter_);
    } else {
      throw MotionPlannerException("Motion planner failed with error code " + std::to_string(prev_result_));
    }

    // ruckig::InputParameter<7> new_input_parameter_;
    // double dt_lookahead = 0.000;
    //
    // auto acc = toEigenD(input_parameter_.current_acceleration);
    // auto vel = toEigenD(input_parameter_.current_velocity);
    // auto pos = toEigenD(input_parameter_.current_position);
    //
    // auto new_vel = vel + acc * dt_lookahead;
    // auto new_pos = pos + (new_vel + vel) / 2 * dt_lookahead;
    //
    // new_input_parameter_.current_position = toStdD<7>(new_pos);
    // new_input_parameter_.current_velocity = toStdD<7>(new_vel);
    // new_input_parameter_.current_acceleration = toStdD<7>(acc);
    // new_input_parameter_.enabled = input_parameter_.enabled;

    return getControlSignal(robot_state, time_step, previous_command, input_parameter_);
  };

  virtual void initWaypointMotion(
      const RobotState &robot_state, const std::optional<ControlSignalType> &previous_command,
      ruckig::InputParameter<7> &input_parameter) = 0;

  virtual void setNewWaypoint(
      const RobotState &robot_state, const std::optional<ControlSignalType> &previous_command,
      const WaypointType &new_waypoint, ruckig::InputParameter<7> &input_parameter) = 0;

  virtual void extrapolateMotion(
      const RobotState &robot_state, const franka::Duration &time_step,
      const ruckig::InputParameter<7> &input_parameter, ruckig::OutputParameter<7> &output_parameter) const = 0;

  virtual void checkWaypoint(const WaypointType &waypoint) const {}

  [[nodiscard]] virtual std::tuple<Vector7d, Vector7d, Vector7d> getStateEstimate(
      const RobotState &robot_state) const = 0;

  [[nodiscard]] virtual std::tuple<Vector7d, Vector7d, Vector7d> getDesiredState(
      const RobotState &robot_state) const = 0;

  [[nodiscard]] virtual std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const = 0;

  [[nodiscard]] virtual std::tuple<Vector7d, Vector7d, Vector7d> getGoalCloseTolerance() const = 0;

  [[nodiscard]] virtual ControlSignalType getControlSignal(
      const RobotState &robot_state, const franka::Duration &time_step,
      const std::optional<ControlSignalType> &previous_command, const ruckig::InputParameter<7> &input_parameter) = 0;

  virtual void setInputLimits(const WaypointType &waypoint, ruckig::InputParameter<7> &input_parameter) const = 0;

 private:
  std::vector<WaypointType> waypoints_;
  bool return_when_finished_{true};
  bool prev_close_to_target_{false};
  bool first_update_{true};

  ruckig::Ruckig<7> trajectory_generator_{Robot::control_rate};
  ruckig::Result prev_result_;
  ruckig::InputParameter<7> input_parameter_;
  ruckig::OutputParameter<7> output_parameter_;

  typename std::vector<WaypointType>::iterator waypoint_iterator_;

  std::optional<franka::Duration> target_reached_time_;
  franka::Duration waypoint_started_time_;

  void checkWaypointPrivate(const WaypointType &waypoint) const {
    if ((waypoint.state_estimate_weight.array() < 0).any() || (waypoint.state_estimate_weight.array() > 1).any()) {
      std::stringstream ss;
      ss << "state_estimate_weight must be between 0 and 1. Got ";
      ss << waypoint.state_estimate_weight;
      throw std::runtime_error(ss.str());
    }
    checkWaypoint(waypoint);
  }
};

}  // namespace franky
