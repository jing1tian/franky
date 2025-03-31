#include "franky/model.hpp"

#include "franky/util.hpp"

namespace franky {

Model::Model(franka::Model model) : model_(std::move(model)) {}

Model::Model(Model &&other) noexcept
    : model_(std::move(other.model_)) {}

Model &Model::operator=(Model &&other) noexcept {
  if (this != &other) {
    model_ = std::move(other.model_);
  }
  return *this;
}

Affine Model::pose(franka::Frame frame, const franka::RobotState &state) const {
  return Affine{Eigen::Map<const Eigen::Matrix4d>(model_.pose(frame, state).data())};
}

Affine Model::pose(franka::Frame frame,
                   const Vector7d &q,
                   const Affine &F_T_EE,
                   const Affine &EE_T_K) const {
  return Affine{Eigen::Map<const Eigen::Matrix4d>(
      model_.pose(frame, toStd<7>(q), toStdMatD<4, 4>(F_T_EE.matrix()), toStdMatD<4, 4>(EE_T_K.matrix())).data())};
}

Jacobian Model::bodyJacobian(franka::Frame frame, const franka::RobotState &state) const {
  return toEigenMatD<6, 7>(model_.bodyJacobian(frame, state));
}

Jacobian Model::bodyJacobian(franka::Frame frame,
                             const Vector7d &q,
                             const Affine &F_T_EE,
                             const Affine &EE_T_K) const {
  return toEigenMatD<6, 7>(
      model_.bodyJacobian(frame, toStd<7>(q), toStdMatD<4, 4>(F_T_EE.matrix()), toStdMatD<4, 4>(EE_T_K.matrix())));
}

Jacobian Model::zeroJacobian(franka::Frame frame, const franka::RobotState &state) const {
  return toEigenMatD<6, 7>(model_.zeroJacobian(frame, state));
}

Jacobian Model::zeroJacobian(franka::Frame frame,
                             const Vector7d &q,
                             const Affine &F_T_EE,
                             const Affine &EE_T_K) const {
  return toEigenMatD<6, 7>(
      model_.zeroJacobian(frame, toStd<7>(q), toStdMatD<4, 4>(F_T_EE.matrix()), toStdMatD<4, 4>(EE_T_K.matrix())));
}

Eigen::Matrix<double, 7, 7> Model::mass(const franka::RobotState &state) const {
  return toEigenMatD<7, 7>(model_.mass(state));
}

Eigen::Matrix<double, 7, 7> Model::mass(const Vector7d &q,
                                        const Eigen::Matrix3d &I_total,
                                        double m_total,
                                        const Eigen::Vector3d &F_x_Ctotal) const {
  return toEigenMatD<7, 7>(model_.mass(toStd<7>(q), toStdMatD<3, 3>(I_total), m_total, toStd<3>(F_x_Ctotal)));
}

Vector7d Model::coriolis(const franka::RobotState &state) const {
  return toEigenD<7>(model_.coriolis(state));
}

Vector7d Model::coriolis(const Vector7d &q,
                         const Vector7d &dq,
                         const Eigen::Matrix3d &I_total,
                         double m_total,
                         const Eigen::Vector3d &F_x_Ctotal) const {
  return toEigenD<7>(model_.coriolis(toStd<7>(q),
                                     toStd<7>(dq),
                                     toStdMatD<3, 3>(I_total),
                                     m_total,
                                     toStd<3>(F_x_Ctotal)));
}

Vector7d Model::gravity(const franka::RobotState &state,
                        const Eigen::Vector3d &gravity_earth) const {
  return toEigenD<7>(model_.gravity(state, toStd<3>(gravity_earth)));
}

Vector7d Model::gravity(const franka::RobotState &state) const {
  return toEigenD<7>(model_.gravity(state));
}

Vector7d Model::gravity(const Vector7d &q,
                        double m_total,
                        const Eigen::Vector3d &F_x_Ctotal,
                        const Eigen::Vector3d &gravity_earth) const {
  return toEigenD<7>(model_.gravity(toStd<7>(q), m_total, toStd<3>(F_x_Ctotal), toStd<3>(gravity_earth)));
}

}  // namespace franky
