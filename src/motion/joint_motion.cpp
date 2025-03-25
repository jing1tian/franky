#include "franky/motion/joint_motion.hpp"

#include "franky/motion/joint_waypoint_motion.hpp"

namespace franky {

JointMotion::JointMotion(
    const JointState &target,
    ReferenceType reference_type,
    RelativeDynamicsFactor relative_dynamics_factor,
    bool return_when_finished)
    : JointWaypointMotion(
    {
        PositionWaypoint<JointState>{
            {
                .target = target,
            },
            reference_type,
        }
    },
    relative_dynamics_factor,
    return_when_finished
) {}

}  // namespace franky
