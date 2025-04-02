#include "franky/motion/joint_velocity_motion.hpp"

#include "franky/motion/joint_velocity_waypoint_motion.hpp"

namespace franky {

JointVelocityMotion::JointVelocityMotion(
    const Vector7d &target,
    const std::optional<franka::Duration> &max_total_duration,
    const RelativeDynamicsFactor &relative_dynamics_factor)
    : JointVelocityWaypointMotion(
    {
        VelocityWaypoint<Vector7d>{
            .target = target,
            .max_total_duration = max_total_duration
        },
        VelocityWaypoint<Vector7d>{
            .target = Vector7d::Zero()
        }
    },
    relative_dynamics_factor
) {}

}  // namespace franky
