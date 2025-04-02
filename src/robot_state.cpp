#include "franky/robot_state.hpp"

#include "franky/util.hpp"

namespace franky {

  RobotState RobotState::from_franka(const franka::RobotState &robot_state) {
    return RobotState{
      .O_T_EE = stdToAffine(robot_state.O_T_EE),
      .O_T_EE_d = stdToAffine(robot_state.O_T_EE_d),
      .F_T_EE = stdToAffine(robot_state.F_T_EE),
      #ifdef FRANKA_0_8
        .F_T_NE = stdToAffine(robot_state.F_T_NE),
        .NE_T_EE = stdToAffine(robot_state.NE_T_EE),
      #endif
      .EE_T_K = stdToAffine(robot_state.EE_T_K),
      .m_ee = robot_state.m_ee,
      .I_ee = toEigenMatD<3, 3>(robot_state.I_ee),
      .F_x_Cee = toEigen(robot_state.F_x_Cee),
      .m_load = robot_state.m_load,
      .I_load = toEigenMatD<3, 3>(robot_state.I_load),
      .F_x_Cload = toEigen(robot_state.F_x_Cload),
      .m_total = robot_state.m_total,
      .I_total = toEigenMatD<3, 3>(robot_state.I_total),
      .F_x_Ctotal = toEigen(robot_state.F_x_Ctotal),
      .elbow = ElbowState(robot_state.elbow),
      .elbow_d = ElbowState(robot_state.elbow_d),
      .elbow_c = ElbowState(robot_state.elbow_c),
      .delbow_c = robot_state.delbow_c[0],
      .ddelbow_c = robot_state.ddelbow_c[0],
      .tau_J = toEigen(robot_state.tau_J),
      .tau_J_d = toEigen(robot_state.tau_J_d),
      .dtau_J = toEigen(robot_state.dtau_J),
      .q = toEigen(robot_state.q),
      .q_d = toEigen(robot_state.q_d),
      .dq = toEigen(robot_state.dq),
      .dq_d = toEigen(robot_state.dq_d),
      .ddq_d = toEigen(robot_state.ddq_d),
      .joint_contact = toEigen(robot_state.joint_contact),
      .cartesian_contact = toEigen(robot_state.cartesian_contact),
      .joint_collision = toEigen(robot_state.joint_collision),
      .cartesian_collision = toEigen(robot_state.cartesian_collision),
      .tau_ext_hat_filtered = toEigen(robot_state.tau_ext_hat_filtered),
      .O_F_ext_hat_K = toEigen(robot_state.O_F_ext_hat_K),
      .K_F_ext_hat_K = toEigen(robot_state.K_F_ext_hat_K),
      .O_dP_EE_d = Twist::fromVectorRepr(toEigen(robot_state.O_dP_EE_d)),
      #ifdef FRANKA_0_8
        .O_ddP_O = toEigen(robot_state.O_ddP_O),
      #endif
      .O_T_EE_c = stdToAffine(robot_state.O_T_EE_c),
      .O_dP_EE_c = Twist::fromVectorRepr(toEigen(robot_state.O_dP_EE_c)),
      .O_ddP_EE_c = TwistAcceleration::fromVectorRepr(toEigen(robot_state.O_ddP_EE_c)),
      .theta = toEigen(robot_state.theta),
      .dtheta = toEigen(robot_state.dtheta),
      .current_errors = robot_state.current_errors,
      .last_motion_errors = robot_state.last_motion_errors,
      .control_command_success_rate = robot_state.control_command_success_rate,
      .robot_mode = robot_state.robot_mode,
      .time = robot_state.time,
    };
  }

}  // namespace franky
