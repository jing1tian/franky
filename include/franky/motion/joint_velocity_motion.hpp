#pragma once

#include "franky/types.hpp"
#include "franky/motion/joint_velocity_waypoint_motion.hpp"

namespace franky {

/**
 * @brief Joint velocity motion with a single target.
 */
class JointVelocityMotion : public JointVelocityWaypointMotion {
 public:
  /**
   * @param target The target joint velocity.
   * @param relative_dynamics_factor The relative dynamics factor for this motion. The factor will get multiplied with
   * the robot's global dynamics factor to get the actual dynamics factor for this motion.
   */
  explicit JointVelocityMotion(const Vector7d &target, const RelativeDynamicsFactor &relative_dynamics_factor = 1.0);
};

}  // namespace franky
