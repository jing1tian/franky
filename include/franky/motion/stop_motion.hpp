#pragma once

#include "franky/motion/reference_type.hpp"
#include "franky/motion/joint_waypoint_motion.hpp"
#include "franky/motion/cartesian_waypoint_motion.hpp"

namespace franky {

template<typename ControlSignalType>
class StopMotion;

/**
 * @brief Stop motion for joint position control mode.
 */
template<>
class StopMotion<franka::JointPositions> : public JointWaypointMotion {
 public:
  explicit StopMotion() : JointWaypointMotion(
      {
          PositionWaypoint<JointState>{
              {
                  .target=JointState(Vector7d::Zero()),
              },
              ReferenceType::Relative
          }
      },
      RelativeDynamicsFactor::MAX_DYNAMICS()
  ) {}
};

/**
 * @brief Stop motion for cartesian pose control mode.
 */
template<>
class StopMotion<franka::CartesianPose> : public CartesianWaypointMotion {
 public:
  explicit StopMotion() : CartesianWaypointMotion(
      {
          PositionWaypoint<CartesianState>{
              {
                  .target = RobotPose(Affine::Identity()),
              },
              ReferenceType::Relative
          }
      },
      RelativeDynamicsFactor::MAX_DYNAMICS()
  ) {}
};

}  // namespace franky