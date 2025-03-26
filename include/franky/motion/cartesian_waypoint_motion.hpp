#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include <franka/robot_state.h>

#include "franky/robot_pose.hpp"
#include "franky/robot.hpp"
#include "franky/motion/reference_type.hpp"
#include "franky/util.hpp"
#include "franky/motion/position_waypoint_motion.hpp"
#include "franky/cartesian_state.hpp"

namespace franky {

/**
 * @brief Cartesian waypoint motion.
 *
 * This motion follows multiple consecutive cartesian targets in a time-optimal way.
 */
class CartesianWaypointMotion : public PositionWaypointMotion<franka::CartesianPose, CartesianState> {
 public:
  /**
   * @param waypoints                Waypoints to follow.
   * @param relative_dynamics_factor The relative dynamics factor for this motion. This factor will get multiplied with
   *                                 the robot's global dynamics factor to get the actual dynamics factor for this
   *                                 motion.
   * @param return_when_finished     Whether to end the motion when the last waypoint is reached or keep holding the
   *                                 last target.
   * @param ee_frame                 The end-effector frame for which the target is defined. This is a transformation
   *                                 from the configured end-effector frame to the end-effector frame the target is
   *                                 defined for.
   */
  explicit CartesianWaypointMotion(
      const std::vector<PositionWaypoint<CartesianState>> &waypoints,
      const RelativeDynamicsFactor &relative_dynamics_factor = 1.0,
      bool return_when_finished = true,
      Affine ee_frame = Affine::Identity());

 protected:
  void initWaypointMotion(
      const franka::RobotState &robot_state,
      const std::optional<franka::CartesianPose> &previous_command,
      ruckig::InputParameter<7> &input_parameter) override;

  void setNewWaypoint(
      const franka::RobotState &robot_state,
      const std::optional<franka::CartesianPose> &previous_command,
      const PositionWaypoint<CartesianState> &new_waypoint,
      ruckig::InputParameter<7> &input_parameter) override;

  [[nodiscard]] std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override;

  [[nodiscard]] franka::CartesianPose getControlSignal(
      const franka::Duration &time_step, const ruckig::InputParameter<7> &input_parameter) override;

 private:
  CartesianState target_state_;
  Affine ref_frame_;
  Affine ee_frame_;

  static inline Vector7d vec_cart_rot_elbow(double cart, double rot, double elbow) {
    return {cart, cart, cart, rot, rot, rot, elbow};
  }
};

}  // namespace franky
