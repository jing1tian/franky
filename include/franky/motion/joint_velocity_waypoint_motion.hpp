#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include <franka/robot_state.h>

#include "franky/joint_state.hpp"
#include "franky/motion/velocity_waypoint_motion.hpp"

namespace franky {

/**
 * @brief Joint velocity waypoint motion.
 *
 * This motion follows multiple joint waypoints in a time-optimal way.
 */
class JointVelocityWaypointMotion : public VelocityWaypointMotion<franka::JointVelocities, Vector7d> {
 public:
  /**
   * @param waypoints Joint waypoints to follow.
   * @param params Parameters for the motion.
   */
  explicit JointVelocityWaypointMotion(
      const std::vector<VelocityWaypoint<Vector7d>> &waypoints,
      const RelativeDynamicsFactor &relative_dynamics_factor = 1.0);

 protected:

  void initWaypointMotion(
      const franka::RobotState &robot_state,
      const std::optional<franka::JointVelocities> &previous_command,
      ruckig::InputParameter<7> &input_parameter) override;

  void setNewWaypoint(
      const franka::RobotState &robot_state,
      const std::optional<franka::JointVelocities> &previous_command,
      const VelocityWaypoint<Vector7d> &new_waypoint,
      ruckig::InputParameter<7> &input_parameter) override;

  [[nodiscard]] std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override;

  [[nodiscard]] franka::JointVelocities getControlSignal(
      const ruckig::InputParameter<7> &input_parameter) const override;
};

}  // namespace franky
