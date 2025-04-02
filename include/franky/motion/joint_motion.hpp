#pragma once

#include "franky/joint_state.hpp"
#include "franky/motion/joint_waypoint_motion.hpp"
#include "franky/motion/reference_type.hpp"

namespace franky {

/**
 * @brief Joint motion with a single target.
 */
class JointMotion : public JointWaypointMotion {
 public:
  /**
   * @param target                   The target joint state.
   * @param state_estimate_weight    Weighting of the robot state estimate vs the target when computing the current
   *                                 state to continue planning from. A value of 0 means that the planner always assumes
   *                                 it reached its last target perfectly (open loop control), while a value of 1 means
   *                                 that the planner always uses the robot state estimate (closed loop control). A
   *                                 value between 0 and 1 means that the planner uses a weighted average of the two.
   * @param relative_dynamics_factor The relative dynamics factor for this motion. The factor will get multiplied with
   *                                 the robot's global dynamics factor to get the actual dynamics factor for this
   *                                 motion.
   * @param return_when_finished     Whether to end the motion when the target is reached or keep holding the last
   *                                 target.
   * @param frame                    The end-effector frame for which the target is defined. This is a transformation
   *                                 from the configured end-effector frame to the end-effector frame the target is
   *                                 defined for.
   */
  explicit JointMotion(
      const JointState &target,
      double state_estimate_weight = 0.0,
      ReferenceType reference_type = ReferenceType::kAbsolute,
      const RelativeDynamicsFactor &relative_dynamics_factor = 1.0,
      bool return_when_finished = true);
};

}  // namespace franky
