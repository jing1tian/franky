#include "franky/motion/cartesian_velocity_motion.hpp"

#include "franky/motion/cartesian_velocity_waypoint_motion.hpp"

namespace franky {

CartesianVelocityMotion::CartesianVelocityMotion(
    const RobotVelocity &target,
    const std::optional<franka::Duration> &max_total_duration,
    const RelativeDynamicsFactor &relative_dynamics_factor,
    const Affine &frame)
    : CartesianVelocityWaypointMotion(
    {
        VelocityWaypoint<RobotVelocity>{
            .target = target,
            .max_total_duration = max_total_duration
        },
        VelocityWaypoint<RobotVelocity>{
            .target = RobotVelocity(),
            .hold_target_duration = franka::Duration(50)
        },
    },
    relative_dynamics_factor,
    frame
) {}

}  // namespace franky
