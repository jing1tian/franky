#include "franky/robot_pose.hpp"

#include <optional>
#include <Eigen/Core>
#include <utility>
#include <franka/control_types.h>

#include "franky/types.hpp"
#include "franky/util.hpp"

namespace franky {

RobotPose::RobotPose(const RobotPose &) = default;

RobotPose::RobotPose(Eigen::Affine3d end_effector_pose, std::optional<ElbowState> elbow_state)
    : end_effector_pose_(std::move(end_effector_pose)), elbow_state_(elbow_state) {}

RobotPose::RobotPose(const Vector7d &vector_repr, bool ignore_elbow, FlipDirection flip_direction)
    : RobotPose(
    vector_repr.head<6>(),
    ignore_elbow ? std::optional<ElbowState>(std::nullopt) : ElbowState(vector_repr[6], flip_direction)) {}

RobotPose::RobotPose(const Vector6d &vector_repr, std::optional<ElbowState> elbow_state)
    : elbow_state_(elbow_state),
      end_effector_pose_(Affine().fromPositionOrientationScale(
          vector_repr.head<3>(),
          Eigen::AngleAxis(vector_repr.tail<3>().norm(), vector_repr.tail<3>().normalized()),
          Eigen::Vector3d::Ones())) {}

RobotPose::RobotPose(const franka::CartesianPose &franka_pose)
    : RobotPose(Affine(Eigen::Matrix4d::Map(franka_pose.O_T_EE.data())), ElbowState(franka_pose.elbow)) {}

Vector7d RobotPose::vector_repr() const {
  Eigen::AngleAxis<double> orientation(end_effector_pose_.rotation());
  auto rotvec = orientation.axis() * orientation.angle();
  double elbow_angle = elbow_state_.has_value() ? elbow_state_.value().joint_3_pos() : 0.0;
  Vector7d result;
  result << end_effector_pose_.translation(), rotvec, elbow_angle;
  return result;
}

franka::CartesianPose RobotPose::as_franka_pose(FlipDirection default_flip_direction) const {
  std::array<double, 16> array = toStd<16>(Eigen::Map<const Eigen::Vector<double, 16>>(end_effector_pose_.data()));
  if (elbow_state_.has_value()) {
    return {array, elbow_state_.value().to_array(default_flip_direction)};
  }
  return {array};
}

RobotPose::RobotPose() : end_effector_pose_(Eigen::Affine3d::Identity()), elbow_state_(std::nullopt) {}

}  // namespace franky
