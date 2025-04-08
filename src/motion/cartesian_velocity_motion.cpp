#include "franky/motion/cartesian_velocity_motion.hpp"

#include "franky/motion/cartesian_velocity_waypoint_motion.hpp"

namespace franky {

CartesianVelocityMotion::CartesianVelocityMotion(
    const RobotVelocity &target,
    const franka::Duration &duration,
    const Eigen::Vector3d &state_estimate_weight,
    const RelativeDynamicsFactor &relative_dynamics_factor,
    const Affine &frame)
    : CartesianVelocityWaypointMotion(
    {
        VelocityWaypoint<RobotVelocity>{
            .target = target,
            .hold_target_duration = duration,
            .max_total_duration = duration,
            .state_estimate_weight = state_estimate_weight,
        },
        VelocityWaypoint<RobotVelocity>{
            .target = RobotVelocity(),
            .hold_target_duration = franka::Duration(50),
            .state_estimate_weight = state_estimate_weight,
        },
    },
    relative_dynamics_factor,
    frame
) {}

}  // namespace franky
