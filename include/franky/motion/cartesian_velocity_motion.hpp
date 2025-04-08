#pragma once

#include "franky/motion/cartesian_velocity_waypoint_motion.hpp"

namespace franky {

/**
 * @brief Cartesian velocity motion with a single target velocity.
 */
class CartesianVelocityMotion : public CartesianVelocityWaypointMotion {
 public:
  /**
   * @brief Construct a Cartesian motion.
   *
   * @param target                   The target Cartesian velocity.
   * @param duration                 Duration this command is active. Default is 1s.
   * @param state_estimate_weight    Weighting of the robot state estimate vs the target when computing the current
   *                                 state to continue planning from. A value of 0 means that the planner always assumes
   *                                 it reached its last target perfectly (open loop control), while a value of 1 means
   *                                 that the planner always uses the robot state estimate (closed loop control). A
   *                                 value between 0 and 1 means that the planner uses a weighted average of the two.
   * @param relative_dynamics_factor The relative dynamics factor for this motion. The factor will get multiplied with
   *                                 the robot's global dynamics factor to get the actual dynamics factor for this
   *                                 motion.
   * @param frame                    The end-effector frame for which the target is defined. This is a transformation
   *                                 from the configured end-effector frame to the end-effector frame the target is
   *                                 defined for.
   */
  explicit CartesianVelocityMotion(
      const RobotVelocity &target,
      const franka::Duration &duration = franka::Duration(1000),
      const Eigen::Vector3d &state_estimate_weight = {0.0, 0.0, 0.0},
      const RelativeDynamicsFactor &relative_dynamics_factor = 1.0,
      const Affine &frame = Affine::Identity());
};

}  // namespace franky
