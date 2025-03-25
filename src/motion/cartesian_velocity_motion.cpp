#include "franky/motion/cartesian_velocity_motion.hpp"

#include "franky/motion/cartesian_velocity_waypoint_motion.hpp"

namespace franky {

CartesianVelocityMotion::CartesianVelocityMotion(
    const RobotVelocity &target,
    const RelativeDynamicsFactor &relative_dynamics_factor,
    const Affine &frame)
    : CartesianVelocityWaypointMotion(
    {
        VelocityWaypoint<RobotVelocity>{
            target
        }
    },
    relative_dynamics_factor,
    frame
) {}

}  // namespace franky
