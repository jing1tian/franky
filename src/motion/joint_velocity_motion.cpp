#include "franky/motion/joint_velocity_motion.hpp"

#include "franky/motion/joint_velocity_waypoint_motion.hpp"

namespace franky {

JointVelocityMotion::JointVelocityMotion(
    const Vector7d &target,
    const franka::Duration &hold_target_duration,
    const RelativeDynamicsFactor &relative_dynamics_factor)
    : JointVelocityWaypointMotion(
    {
        VelocityWaypoint<Vector7d>{
            .target = target,
            .hold_target_duration = hold_target_duration
        }
    },
    relative_dynamics_factor
) {}

}  // namespace franky
