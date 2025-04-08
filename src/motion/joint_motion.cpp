#include "franky/motion/joint_motion.hpp"

#include "franky/motion/joint_waypoint_motion.hpp"

namespace franky {

JointMotion::JointMotion(
    const JointState &target,
    const Eigen::Vector3d &state_estimate_weight,
    ReferenceType reference_type,
    const RelativeDynamicsFactor &relative_dynamics_factor,
    bool return_when_finished)
    : JointWaypointMotion(
    {
        PositionWaypoint<JointState>{
            {
                .target = target,
                .state_estimate_weight = state_estimate_weight
            },
            reference_type,
        }
    },
    relative_dynamics_factor,
    return_when_finished
) {}

}  // namespace franky
