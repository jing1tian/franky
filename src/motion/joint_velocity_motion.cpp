#include "franky/motion/joint_velocity_motion.hpp"

#include "franky/motion/joint_velocity_waypoint_motion.hpp"

namespace franky {

JointVelocityMotion::JointVelocityMotion(
    const Vector7d &target,
    const RelativeDynamicsFactor &relative_dynamics_factor)
    : JointVelocityWaypointMotion(
    {
        VelocityWaypoint<Vector7d>{target}
    },
    relative_dynamics_factor
) {}

}  // namespace franky
