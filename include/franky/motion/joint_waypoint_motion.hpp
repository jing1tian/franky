#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include <franka/robot_state.h>

#include "franky/joint_state.hpp"
#include "franky/motion/position_waypoint_motion.hpp"

namespace franky {

/**
 * @brief Joint waypoint motion.
 *
 * This motion follows multiple joint waypoints in a time-optimal way.
 */
class JointWaypointMotion : public PositionWaypointMotion<franka::JointPositions, JointState> {
 public:
  /**
   * @param waypoints Joint waypoints to follow.
   * @param params Parameters for the motion.
   */
  explicit JointWaypointMotion(
      const std::vector<PositionWaypoint<JointState>> &waypoints,
      const RelativeDynamicsFactor relative_dynamics_factor = 1.0,
      bool return_when_finished = true);

 protected:

  void initWaypointMotion(
      const franka::RobotState &robot_state,
      const std::optional<franka::JointPositions> &previous_command,
      ruckig::InputParameter<7> &input_parameter) override;

  void setNewWaypoint(
      const franka::RobotState &robot_state,
      const std::optional<franka::JointPositions> &previous_command,
      const PositionWaypoint<JointState> &new_waypoint,
      ruckig::InputParameter<7> &input_parameter) override;

  [[nodiscard]] std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override;

  [[nodiscard]] franka::JointPositions getControlSignal(
      const ruckig::InputParameter<7> &input_parameter) const override;
};

}  // namespace franky
