#include "franky/model.hpp"

#include "franky/util.hpp"

namespace franky {

Model::Model(franka::Model model) : model_(std::move(model)) {}

Model::Model(Model &&other) noexcept : model_(std::move(other.model_)) {}

Model &Model::operator=(Model &&other) noexcept {
  if (this != &other) {
    model_ = std::move(other.model_);
  }
  return *this;
}

Affine Model::pose(franka::Frame frame, const RobotState &state) const {
  return pose(frame, state.q, state.F_T_EE, state.EE_T_K);
}

Affine Model::pose(franka::Frame frame, const Vector7d &q, const Affine &F_T_EE, const Affine &EE_T_K) const {
  return Affine{Eigen::Map<const Eigen::Matrix4d>(
      model_.pose(frame, toStdD<7>(q), toStdDMatD<4, 4>(F_T_EE.matrix()), toStdDMatD<4, 4>(EE_T_K.matrix())).data())};
}

Jacobian Model::bodyJacobian(franka::Frame frame, const RobotState &state) const {
  return bodyJacobian(frame, state.q, state.F_T_EE, state.EE_T_K);
}

Jacobian Model::bodyJacobian(franka::Frame frame, const Vector7d &q, const Affine &F_T_EE, const Affine &EE_T_K) const {
  return toEigenMatD<6, 7>(
      model_.bodyJacobian(frame, toStdD<7>(q), toStdDMatD<4, 4>(F_T_EE.matrix()), toStdDMatD<4, 4>(EE_T_K.matrix())));
}

Jacobian Model::zeroJacobian(franka::Frame frame, const RobotState &state) const {
  return zeroJacobian(frame, state.q, state.F_T_EE, state.EE_T_K);
}

Jacobian Model::zeroJacobian(franka::Frame frame, const Vector7d &q, const Affine &F_T_EE, const Affine &EE_T_K) const {
  return toEigenMatD<6, 7>(
      model_.zeroJacobian(frame, toStdD<7>(q), toStdDMatD<4, 4>(F_T_EE.matrix()), toStdDMatD<4, 4>(EE_T_K.matrix())));
}

Eigen::Matrix<double, 7, 7> Model::mass(const RobotState &state) const {
  return mass(state.q, state.I_total, state.m_total, state.F_x_Ctotal);
}

Eigen::Matrix<double, 7, 7> Model::mass(
    const Vector7d &q, const Eigen::Matrix3d &I_total, double m_total, const Eigen::Vector3d &F_x_Ctotal) const {
  return toEigenMatD<7, 7>(model_.mass(toStdD<7>(q), toStdDMatD<3, 3>(I_total), m_total, toStdD<3>(F_x_Ctotal)));
}

Vector7d Model::coriolis(const RobotState &state) const {
  return coriolis(state.q, state.dq, state.I_total, state.m_total, state.F_x_Ctotal);
}

Vector7d Model::coriolis(
    const Vector7d &q, const Vector7d &dq, const Eigen::Matrix3d &I_total, double m_total,
    const Eigen::Vector3d &F_x_Ctotal) const {
  return toEigenD<7>(
      model_.coriolis(toStdD<7>(q), toStdD<7>(dq), toStdDMatD<3, 3>(I_total), m_total, toStdD<3>(F_x_Ctotal)));
}

Vector7d Model::gravity(const RobotState &state, const Eigen::Vector3d &gravity_earth) const {
  return gravity(state.q, state.m_total, state.F_x_Ctotal, gravity_earth);
}

Vector7d Model::gravity(const RobotState &state) const {
#ifdef FRANKA_0_9
  return gravity(state, state.O_ddP_O);
#else
  franka::RobotState franka_robot_state;
  franka_robot_state.q = toStdD<7>(state.q);
  return toEigenD(model_.gravity(franka_robot_state));
#endif
}

Vector7d Model::gravity(
    const Vector7d &q, double m_total, const Eigen::Vector3d &F_x_Ctotal, const Eigen::Vector3d &gravity_earth) const {
  return toEigenD<7>(model_.gravity(toStdD<7>(q), m_total, toStdD<3>(F_x_Ctotal), toStdD<3>(gravity_earth)));
}

}  // namespace franky
