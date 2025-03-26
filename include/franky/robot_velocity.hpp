#pragma once

#include <optional>
#include <Eigen/Core>
#include <franka/control_types.h>

#include "franky/types.hpp"
#include "franky/twist.hpp"

namespace franky {

/**
 * @brief Cartesian velocity of a robot.
 *
 * This class encapsulates the cartesian velocity of a robot, which comproses the end effector twist and the elbow
 * velocity.
 */
class RobotVelocity {
 public:
  RobotVelocity();

  RobotVelocity(const RobotVelocity &robot_velocity);

  // Suppress implicit conversion warning
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wimplicit-conversion"
  /**
   * @param end_effector_twist The twist of the end effector.
   * @param elbow The velocity of the elbow. Default is 0.0.
   */
  RobotVelocity(const Twist &end_effector_twist, std::optional<double> elbow = 0.0);
#pragma clang diagnostic pop

  /**
   * @param vector_repr  The vector representation of the velocity.
   * @param ignore_elbow Whether to ignore the elbow velocity. Default is false.
   */
  explicit RobotVelocity(const Vector7d &vector_repr, bool ignore_elbow = false);

  /**
 * @param vector_repr The vector representation of the velocity.
 * @param elbow The velocity of the elbow. Optional.
 */
  explicit RobotVelocity(const Vector6d &vector_repr, std::optional<double> elbow = std::nullopt);

  /**
   * @param franka_velocity The franka velocity.
   */
  explicit RobotVelocity(franka::CartesianVelocities franka_velocity);

  /**
   * @brief Get the vector representation of the velocity. It consists of the linear and angular velocity of the end
   * effector and the joint velocity of the elbow.
   *
   * @return The vector representation of the velocity.
   */
  [[nodiscard]] Vector7d vector_repr() const;

  /**
   * @brief Get the franka velocity.
   *
   * @return The franka velocity.
   */
  [[nodiscard]] franka::CartesianVelocities as_franka_velocity() const;

  /**
   * @brief Transform the frame of the velocity by applying the given affine transform.
   *
   * @param affine The affine to apply.
   * @return The velocity after the transformation.
   */
  [[nodiscard]] inline RobotVelocity transform(const Affine &affine) const {
    return transform(affine.rotation());
  }

  /**
   * @brief Transform the frame of the velocity by applying the given rotation.
   *
   * @param rotation The rotation to apply.
   * @return The velocity after the transformation.
   */
  template<typename RotationMatrixType>
  [[nodiscard]] inline RobotVelocity transform(const RotationMatrixType &rotation) const {
    return {rotation * end_effector_twist_, elbow_};
  }

  /**
   * @brief Change the end-effector frame by adding the given offset to the current end-effector frame. Note that the
   * offset must be given in world coordinates.
   *
   * @param offset_world_frame The offset to add to the current end-effector frame.
   * @return The velocity of the new end-effector frame.
   */
  [[nodiscard]] inline RobotVelocity changeEndEffectorFrame(const Eigen::Vector3d &offset_world_frame) const {
    return {end_effector_twist_.propagateThroughLink(offset_world_frame), elbow_};
  }

  /**
   * @brief Get the velocity with a new elbow velocity.
   *
   * @param elbow The new elbow velocity / position.
   * @return The velocity with the new elbow velocity.
   */
    [[nodiscard]] inline RobotVelocity withElbow(const std::optional<double> elbow) const {
      return {end_effector_twist_, elbow};
    }

  /**
   * @brief Get the end effector twist.
   *
   * @return The end effector twist.
   */
  [[nodiscard]] inline Twist end_effector_twist() const {
    return end_effector_twist_;
  }

  /**
   * @brief Get the elbow velocity/position. If this object is retrieved by reading the franka robot state, then
   * this field will contain the elbow velocity. If this object is created by the user for cartesian velocity control,
   * then it must contain the elbow position.
   *
   * @return The elbow velocity/position.
   */
  [[nodiscard]] inline std::optional<double> elbow() const {
    return elbow_;
  }

 private:
  Twist end_effector_twist_;
  std::optional<double> elbow_ = 0.0;
};

inline RobotVelocity operator*(const Affine &affine, const RobotVelocity &robot_velocity) {
  return robot_velocity.transform(affine);
}

template<typename RotationMatrixType>
inline RobotVelocity operator*(const RotationMatrixType &rotation, const RobotVelocity &robot_velocity) {
  return robot_velocity.transform(rotation);
}

}  // namespace franky
