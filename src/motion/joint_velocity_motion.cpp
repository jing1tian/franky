#include "franky/motion/joint_velocity_motion.hpp"

#include "franky/motion/joint_velocity_waypoint_motion.hpp"

namespace franky {

JointVelocityMotion::JointVelocityMotion(
    const Vector7d &target,
    const franka::Duration &duration,
    double state_estimate_weight,
    const RelativeDynamicsFactor &relative_dynamics_factor)
    : JointVelocityWaypointMotion(
    {
        VelocityWaypoint<Vector7d>{
            .target = target,
            .hold_target_duration = duration,
            .max_total_duration = duration,
            .state_estimate_weight = state_estimate_weight
        },
        VelocityWaypoint<Vector7d>{
            .target = Vector7d::Zero(),
            .state_estimate_weight = state_estimate_weight
        }
    },
    relative_dynamics_factor
) {}

}  // namespace franky
