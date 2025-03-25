#include "franky/motion/cartesian_motion.hpp"

#include "franky/robot_pose.hpp"
#include "franky/motion/cartesian_waypoint_motion.hpp"

namespace franky {

CartesianMotion::CartesianMotion(
    const CartesianState &target,
    ReferenceType reference_type,
    const RelativeDynamicsFactor &relative_dynamics_factor,
    bool return_when_finished,
    const Affine &frame)
    : CartesianWaypointMotion(
    {
        PositionWaypoint<CartesianState>{
            {
                .target = target,
            },
            reference_type
        }
    },
    relative_dynamics_factor,
    return_when_finished,
    frame
) {}

}  // namespace franky
