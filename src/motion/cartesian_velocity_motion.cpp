#include "franky/motion/cartesian_velocity_motion.hpp"

#include "franky/motion/cartesian_velocity_waypoint_motion.hpp"

namespace franky {

CartesianVelocityMotion::CartesianVelocityMotion(
    const RobotVelocity &target,
    const franka::Duration &hold_target_duration,
    const RelativeDynamicsFactor &relative_dynamics_factor,
    const Affine &frame)
    : CartesianVelocityWaypointMotion(
    {
        VelocityWaypoint<RobotVelocity>{
            .target = target,
            .hold_target_duration = hold_target_duration
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
