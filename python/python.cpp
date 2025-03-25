#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <Python.h>

#include <franka/robot_state.h>

#include "franky.hpp"

#include "sequential_executor.hpp"
#include "macros.hpp"

#define ADD_FIELD_RO(obj_type, unused, name) .def_readonly(#name, &obj_type::name)
#define ADD_FIELDS_RO(obj, obj_type, ...) obj MAP_C1(ADD_FIELD_RO, obj_type, __VA_ARGS__);

#define COUNT_INNER(i, unused) i + 1
#define COUNT(...) FOLD(COUNT_INNER, 0, __VA_ARGS__)

#define PACK_TUPLE_INNER(obj, unused, name) , obj.name
#define PACK_TUPLE_1(obj, name0, ...) py::make_tuple(obj.name0 MAP_C1(PACK_TUPLE_INNER, obj, __VA_ARGS__))
#define PACK_TUPLE(obj, ...) PACK_TUPLE_1(obj, __VA_ARGS__)

#define UNPACK_TUPLE_INNER(obj_type, tuple, itr, name) , .name = tuple[itr + 1].cast<decltype(obj_type::name)>()
#define UNPACK_TUPLE_1(obj_type, tuple, name0, ...) obj_type{.name0 = tuple[0].cast<decltype(obj_type::name0)>() MAP_C2(UNPACK_TUPLE_INNER, obj_type, tuple, __VA_ARGS__)}
#define UNPACK_TUPLE(obj_type, tuple, ...) UNPACK_TUPLE_1(obj_type, tuple, __VA_ARGS__)

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace franky;

SequentialExecutor callback_executor;

template<int dims>
std::string vecToStr(const Eigen::Vector<double, dims> &vec) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < dims; i++) {
    ss << vec[i];
    if (i != dims - 1)
      ss << " ";
  }
  ss << "]";
  return ss.str();
}

std::string affineToStr(const Affine &affine) {
  std::stringstream ss;
  ss << "Affine(t=" << vecToStr(affine.translation().eval())
     << ", q=" << vecToStr(Eigen::Quaterniond(affine.rotation()).coeffs()) << ")";
  return ss.str();
}

std::string twistToStr(const Twist &twist) {
  std::stringstream ss;
  ss << "Twist(lin=" << vecToStr(twist.linear_velocity())
     << ", ang=" << vecToStr(twist.angular_velocity()) << ")";
  return ss.str();
}

std::string robotPoseToStr(const RobotPose &robot_pose) {
  std::stringstream ss;
  ss << "RobotPose(ee_pose=" << affineToStr(robot_pose.end_effector_pose());
  if (robot_pose.elbow_position().has_value())
    ss << ", elbow=" << robot_pose.elbow_position().value();
  ss << ")";
  return ss.str();
}

std::string robotVelocityToStr(const RobotVelocity &robot_velocity) {
  std::stringstream ss;
  ss << "RobotVelocity(ee_twist=" << twistToStr(robot_velocity.end_effector_twist());
  if (robot_velocity.elbow_velocity().has_value())
    ss << ", elbow_vel=" << robot_velocity.elbow_velocity().value();
  ss << ")";
  return ss.str();
}

template<typename ControlSignalType>
void mkMotionAndReactionClasses(py::module_ m, const std::string &control_signal_name) {
  py::class_<Motion<ControlSignalType>, std::shared_ptr<Motion<ControlSignalType>>> motion_class(
      m, (control_signal_name + "Motion").c_str());
  py::class_<Reaction<ControlSignalType>, std::shared_ptr<Reaction<ControlSignalType>>> reaction_class(
      m, (control_signal_name + "Reaction").c_str());

  motion_class
      .def_property_readonly("reactions", &Motion<ControlSignalType>::reactions)
      .def("add_reaction", &Motion<ControlSignalType>::addReaction)
      .def("register_callback", [](
          Motion<ControlSignalType> &motion,
          const typename Motion<ControlSignalType>::CallbackType &callback
      ) {
        motion.registerCallback([callback](
            const franka::RobotState &robot_state,
            franka::Duration time_step,
            std::chrono::duration<double> rel_time,
            std::chrono::duration<double> abs_time,
            const ControlSignalType &control_signal) {
          callback_executor.add([callback, robot_state, time_step, rel_time, abs_time, control_signal]() {
            try {
              callback(robot_state, time_step, rel_time, abs_time, control_signal);
            } catch (const py::error_already_set &e) {
              std::cerr << "Error in callback: " << std::endl;
              py::gil_scoped_acquire gil_acquire;
              PyErr_Print();
            }
          });
        });
      }, "callback"_a);

  reaction_class
      .def(py::init<const Condition &, std::shared_ptr<Motion<ControlSignalType>>>(),
           "condition"_a, "motion"_a = nullptr)
      .def("register_callback", [](
          Reaction<ControlSignalType> &reaction,
          const std::function<void(
              const franka::RobotState &, std::chrono::duration<double>, std::chrono::duration<double>)> &callback
      ) {
        reaction.registerCallback([callback](
            const franka::RobotState &robot_state,
            std::chrono::duration<double> rel_time,
            std::chrono::duration<double> abs_time) {
          callback_executor.add([callback, robot_state, rel_time, abs_time]() {
            try {
              callback(robot_state, rel_time, abs_time);
            } catch (const py::error_already_set &e) {
              py::gil_scoped_acquire gil_acquire;
              std::cerr << "Error in callback: ";
              PyErr_Print();
            }
          });
        });
      }, "callback"_a);
}

template<typename ControlSignalType>
void robotMove(Robot &robot, std::shared_ptr<Motion<ControlSignalType>> motion, bool async) {
  robot.move(motion, true);
  if (!async) {
    auto future = std::async(std::launch::async, (bool (Robot::*)()) &Robot::joinMotion, &robot);
    // Check if python wants to terminate every 100 ms
    bool python_terminating = false;
    while (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout) {
      {
        py::gil_scoped_acquire gil_acquire;
        python_terminating = Py_IsInitialized() && PyErr_CheckSignals() == -1;
      }
      if (python_terminating) {
        robot.stop();
        future.wait();
        py::gil_scoped_acquire gil_acquire;
        throw py::error_already_set();
      }
    }
    future.get();
  }
}

PYBIND11_MODULE(_franky, m) {
  m.doc() = "High-Level Motion Library for the Franka Panda Robot";

  py::enum_<ReferenceType>(m, "ReferenceType")
      .value("Relative", ReferenceType::Relative)
      .value("Absolute", ReferenceType::Absolute);

  py::enum_<franka::ControllerMode>(m, "ControllerMode")
      .value("JointImpedance", franka::ControllerMode::kJointImpedance)
      .value("CartesianImpedance", franka::ControllerMode::kCartesianImpedance);

  py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
      .value("Enforce", franka::RealtimeConfig::kEnforce)
      .value("Ignore", franka::RealtimeConfig::kIgnore);

  py::enum_<ControlSignalType>(m, "ControlSignalType")
      .value("Torques", ControlSignalType::Torques)
      .value("JointVelocities", ControlSignalType::JointVelocities)
      .value("JointPositions", ControlSignalType::JointPositions)
      .value("CartesianVelocities", ControlSignalType::CartesianVelocities)
      .value("CartesianPose", ControlSignalType::CartesianPose);

  py::enum_<franka::RobotMode>(m, "RobotMode")
      .value("Other", franka::RobotMode::kOther)
      .value("Idle", franka::RobotMode::kIdle)
      .value("Move", franka::RobotMode::kMove)
      .value("Guiding", franka::RobotMode::kGuiding)
      .value("Reflex", franka::RobotMode::kReflex)
      .value("UserStopped", franka::RobotMode::kUserStopped)
      .value("AutomaticErrorRecovery", franka::RobotMode::kAutomaticErrorRecovery);

  py::class_<franka::Duration>(m, "Duration")
      .def(py::init<>())
      .def(py::init<uint64_t>())
      .def("to_sec", &franka::Duration::toSec)
      .def("to_msec", &franka::Duration::toMSec)
      .def(py::self + py::self)
      .def(py::self += py::self)
      .def(py::self - py::self)
      .def(py::self -= py::self)
      .def(py::self * uint64_t())
      .def(py::self *= uint64_t())
      .def(py::self / uint64_t())
      .def(py::self /= uint64_t())
      .def("__repr__",
           [](const franka::Duration &duration) {
             std::stringstream ss;
             ss << "Duration(" << duration.toMSec() << ")";
             return ss.str();
           }
      )
      .def(py::pickle(
          [](const franka::Duration &duration) {  // __getstate__
            return py::make_tuple(duration.toMSec());
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 1)
              throw std::runtime_error("Invalid state!");
            return franka::Duration(t[0].cast<uint64_t>());
          }
      ));

  py::class_<RelativeDynamicsFactor>(m, "RelativeDynamicsFactor")
      .def(py::init<>())
      .def(py::init<double>(), "value"_a)
      .def(py::init<double, double, double>(), "velocity"_a, "acceleration"_a, "jerk"_a)
      .def_property_readonly("velocity", &RelativeDynamicsFactor::velocity)
      .def_property_readonly("acceleration", &RelativeDynamicsFactor::acceleration)
      .def_property_readonly("jerk", &RelativeDynamicsFactor::jerk)
      .def_property_readonly("max_dynamics", &RelativeDynamicsFactor::max_dynamics)
      .def_property_readonly_static("MAX_DYNAMICS", [](py::object) { return RelativeDynamicsFactor::MAX_DYNAMICS(); })
      .def(py::self * py::self);
  py::implicitly_convertible<double, RelativeDynamicsFactor>();

#define ERRORS_0_8 \
  joint_position_limits_violation, \
  cartesian_position_limits_violation, \
  self_collision_avoidance_violation, \
  joint_velocity_violation, \
  cartesian_velocity_violation, \
  force_control_safety_violation, \
  joint_reflex, \
  cartesian_reflex, \
  max_goal_pose_deviation_violation, \
  max_path_pose_deviation_violation, \
  cartesian_velocity_profile_safety_violation, \
  joint_position_motion_generator_start_pose_invalid, \
  joint_motion_generator_position_limits_violation, \
  joint_motion_generator_velocity_limits_violation, \
  joint_motion_generator_velocity_discontinuity, \
  joint_motion_generator_acceleration_discontinuity, \
  cartesian_position_motion_generator_start_pose_invalid, \
  cartesian_motion_generator_elbow_limit_violation, \
  cartesian_motion_generator_velocity_limits_violation, \
  cartesian_motion_generator_velocity_discontinuity, \
  cartesian_motion_generator_acceleration_discontinuity, \
  cartesian_motion_generator_elbow_sign_inconsistent, \
  cartesian_motion_generator_start_elbow_invalid, \
  cartesian_motion_generator_joint_position_limits_violation, \
  cartesian_motion_generator_joint_velocity_limits_violation, \
  cartesian_motion_generator_joint_velocity_discontinuity, \
  cartesian_motion_generator_joint_acceleration_discontinuity, \
  cartesian_position_motion_generator_invalid_frame, \
  force_controller_desired_force_tolerance_violation, \
  controller_torque_discontinuity, \
  start_elbow_sign_inconsistent, \
  communication_constraints_violation, \
  power_limit_violation, \
  joint_p2p_insufficient_torque_for_planning, \
  tau_j_range_violation, \
  instability_detected, \
  joint_move_in_wrong_direction

#ifdef FRANKA_0_9
#define ERRORS ERRORS_0_8, \
  cartesian_spline_motion_generator_violation, \
  joint_via_motion_generator_planning_joint_limit_violation, \
  base_acceleration_initialization_timeout, \
  base_acceleration_invalid_reading
#define NUM_ERRORS 41
#else
#define ERRORS ERRORS_0_8
#define NUM_ERRORS 37
#endif

#define ADD_ERROR(unused, name) errors.def_property_readonly(#name, [](const franka::Errors &e) { return e.name; });
#define UNPACK_ERRORS_INNER(tuple, itr, name) , tuple[itr + 1].cast<bool>()
#define UNPACK_ERRORS_1(tuple, name0, ...) franka::Errors{std::array<bool, NUM_ERRORS>{tuple[0].cast<bool>() MAP_C1(UNPACK_ERRORS_INNER, tuple, __VA_ARGS__)}}
#define UNPACK_ERRORS(tuple, ...) UNPACK_ERRORS_1(tuple, __VA_ARGS__)

  py::class_<franka::Errors> errors(m, "Errors");
  errors.def(py::init<>());
  MAP(ADD_ERROR, ERRORS)
  errors.def("__repr__", [](const franka::Errors &errors) { return std::string(errors); });
  errors.def(py::pickle(
      [](const franka::Errors &errors) {  // __getstate__
        return PACK_TUPLE(errors, ERRORS);
      },
      [](const py::tuple &t) {  // __setstate__
        if (t.size() != COUNT(ERRORS))
          throw std::runtime_error("Invalid state!");
        return UNPACK_ERRORS(t, ERRORS);
      }
  ));

  py::class_<Condition>(m, "Condition")
      .def(py::init<bool>(), "constant_value"_a)
      .def("__invert__", py::overload_cast<const Condition &>(&operator!), py::is_operator())
      .def(py::self == py::self)
      .def(py::self == bool())
      .def(py::self != py::self)
      .def(py::self != bool())
      .def("__and__", py::overload_cast<const Condition &, const Condition &>(&operator&&), py::is_operator())
      .def("__and__",
           [](const Condition &condition, bool constant) { return condition && constant; },
           py::is_operator())
      .def("__rand__",
           [](bool constant, const Condition &condition) { return constant && condition; },
           py::is_operator())
      .def("__or__", py::overload_cast<const Condition &, const Condition &>(&operator||), py::is_operator())
      .def("__or__", [](const Condition &condition, bool constant) { return condition || constant; }, py::is_operator())
      .def("__ror__",
           [](bool constant, const Condition &condition) { return constant || condition; },
           py::is_operator())
      .def("__repr__", &Condition::repr);
  py::implicitly_convertible<bool, Condition>();

  py::class_<Measure>(m, "Measure")
      .def_property_readonly_static("FORCE_X", [](py::object) { return Measure::ForceX(); })
      .def_property_readonly_static("FORCE_Y", [](py::object) { return Measure::ForceY(); })
      .def_property_readonly_static("FORCE_Z", [](py::object) { return Measure::ForceZ(); })
      .def_property_readonly_static("REL_TIME", [](py::object) { return Measure::RelTime(); })
      .def_property_readonly_static("ABS_TIME", [](py::object) { return Measure::AbsTime(); })
      .def(-py::self)
      .def(py::self == py::self)
      .def(py::self == double_t())
      .def(py::self != py::self)
      .def(py::self != double_t())
      .def(py::self > py::self)
      .def(py::self > double_t())
      .def(py::self >= py::self)
      .def(py::self >= double_t())
      .def(py::self < py::self)
      .def(py::self < double_t())
      .def(py::self <= py::self)
      .def(py::self <= double_t())
      .def(py::self + py::self)
      .def(py::self + double_t())
      .def(double_t() + py::self)
      .def(py::self - py::self)
      .def(py::self - double_t())
      .def(double_t() - py::self)
      .def(py::self * py::self)
      .def(py::self * double_t())
      .def(double_t() * py::self)
      .def(py::self / py::self)
      .def(py::self / double_t())
      .def(double_t() / py::self)
      .def("__pow__", py::overload_cast<const Measure &, const Measure &>(&measure_pow), py::is_operator())
      .def("__pow__",
           [](const Measure &measure, double constant) { return measure_pow(measure, constant); },
           py::is_operator())
      .def("__rpow__",
           [](const Measure &measure, double constant) { return measure_pow(constant, measure); },
           py::is_operator())
      .def("__repr__", &Measure::repr);

#ifdef FRANKA_0_8
#define EXTRA_FIELDS1 \
    F_T_NE, \
    NE_T_EE,
#else
#define EXTRA_FIELDS1
#endif

#ifdef FRANKA_0_9
#define EXTRA_FIELDS2 \
    O_ddP_O,
#else
#define EXTRA_FIELDS2
#endif

#define ROBOT_STATE_FIELDS \
    O_T_EE, \
    O_T_EE_d, \
    F_T_EE, \
    EXTRA_FIELDS1 \
    EE_T_K, \
    m_ee, \
    I_ee, \
    F_x_Cee, \
    m_load, \
    I_load, \
    F_x_Cload, \
    m_total, \
    I_total, \
    F_x_Ctotal, \
    elbow, \
    elbow_d, \
    elbow_c, \
    delbow_c, \
    ddelbow_c, \
    tau_J, \
    tau_J_d, \
    dtau_J, \
    q, \
    q_d, \
    dq, \
    dq_d, \
    ddq_d, \
    joint_contact, \
    cartesian_contact, \
    joint_collision, \
    cartesian_collision, \
    tau_ext_hat_filtered, \
    O_F_ext_hat_K, \
    K_F_ext_hat_K, \
    O_dP_EE_d, \
    EXTRA_FIELDS2 \
    O_T_EE_c, \
    O_dP_EE_c, \
    O_ddP_EE_c, \
    theta, \
    dtheta, \
    current_errors, \
    last_motion_errors, \
    control_command_success_rate, \
    robot_mode, \
    time

  py::class_<franka::RobotState> robot_state(m, "RobotState");
  ADD_FIELDS_RO(robot_state, franka::RobotState, ROBOT_STATE_FIELDS)
  robot_state.def(py::pickle(
      [](const franka::RobotState &state) {  // __getstate__
        return PACK_TUPLE(state, ROBOT_STATE_FIELDS);
      },
      [](const py::tuple &t) {  // __setstate__
        if (t.size() != COUNT(ROBOT_STATE_FIELDS))
          throw std::runtime_error("Invalid state!");
        return UNPACK_TUPLE(franka::RobotState, t, ROBOT_STATE_FIELDS);
      }
  ));

#define GRIPPER_STATE_FIELDS width, max_width, is_grasped, temperature, time

  py::class_<franka::GripperState> gripper_state(m, "GripperState");
  ADD_FIELDS_RO(gripper_state, franka::GripperState, GRIPPER_STATE_FIELDS)
  gripper_state.def(py::pickle(
      [](const franka::GripperState &state) {  // __getstate__
        return PACK_TUPLE(state, GRIPPER_STATE_FIELDS);
      },
      [](const py::tuple &t) {  // __setstate__
        if (t.size() != COUNT(GRIPPER_STATE_FIELDS))
          throw std::runtime_error("Invalid state!");
        return UNPACK_TUPLE(franka::GripperState, t, GRIPPER_STATE_FIELDS);
      }
  ));

  py::class_<Affine>(m, "Affine")
      .def(py::init<const Eigen::Matrix<double, 4, 4> &>(),
           "transformation_matrix"_a = Eigen::Matrix<double, 4, 4>::Identity())
      .def(py::init<>([](const Eigen::Vector3d &translation, const Eigen::Vector4d &quaternion) {
        return Affine().fromPositionOrientationScale(
            translation, Eigen::Quaterniond(quaternion), Eigen::Vector3d::Ones());
      }), "translation"_a = Eigen::Vector3d{0, 0, 0}, "quaternion"_a = Eigen::Vector4d{0, 0, 0, 1})
      .def(py::init<const Affine &>()) // Copy constructor
      .def(py::self * py::self)
      .def_property_readonly("inverse", [](const Affine &affine) { return affine.inverse(); })
      .def_property_readonly("translation", [](const Affine &affine) {
        return affine.translation();
      })
      .def_property_readonly("quaternion", [](const Affine &affine) {
        return Eigen::Quaterniond(affine.rotation()).coeffs();
      })
      .def_property_readonly("matrix", [](const Affine &affine) {
        return affine.matrix();
      })
      .def("__repr__", &affineToStr)
      .def(py::pickle(
          [](const Affine &affine) {  // __getstate__
            return py::make_tuple(affine.translation(), Eigen::Quaterniond(affine.rotation()).coeffs());
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            return Affine().fromPositionOrientationScale(
                t[0].cast<Eigen::Vector3d>(),
                Eigen::Quaterniond(t[1].cast<Eigen::Vector4d>()), Eigen::Vector3d::Ones());
          }
      ));

  py::class_<Twist>(m, "Twist")
      .def(py::init([](
          const std::optional<Eigen::Vector3d> &linear_velocity,
          const std::optional<Eigen::Vector3d> &angular_velocity) {
        return Twist(
            linear_velocity.value_or(Eigen::Vector3d::Zero()), angular_velocity.value_or(Eigen::Vector3d::Zero()));
      }), "linear_velocity"_a = std::nullopt, "angular_velocity"_a = std::nullopt)
      .def(py::init<const Twist &>()) // Copy constructor
      .def("propagate_through_link", &Twist::propagateThroughLink, "link_translation"_a)
      .def("transform_with",
           [](const Twist &twist, const Affine &affine) { return twist.transformWith(affine); },
           "affine"_a)
      .def("transform_with",
           [](const Twist &twist, const Eigen::Quaterniond &quaterion) { return twist.transformWith(quaterion); },
           "quaternion"_a)
      .def_property_readonly("linear", &Twist::linear_velocity)
      .def_property_readonly("angular", &Twist::angular_velocity)
      .def("__rmul__",
           [](const Twist &twist, const Affine &affine) { return affine * twist; },
           py::is_operator())
      .def("__rmul__",
           [](const Twist &twist, const Eigen::Quaterniond &quaternion) { return quaternion * twist; },
           py::is_operator())
      .def("__repr__", &twistToStr)
      .def(py::pickle(
          [](const Twist &twist) {  // __getstate__
            return py::make_tuple(twist.linear_velocity(), twist.angular_velocity());
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            return Twist(t[0].cast<Eigen::Vector3d>(), t[1].cast<Eigen::Vector3d>());
          }
      ));

  py::class_<RobotPose>(m, "RobotPose")
      .def(py::init<Affine, std::optional<double>>(),
           "end_effector_pose"_a,
           "elbow_position"_a = std::nullopt)
      .def(py::init<const RobotPose &>()) // Copy constructor
      .def("with_elbow_position", &RobotPose::withElbowPosition, "elbow_position"_a)
      .def_property_readonly("end_effector_pose", &RobotPose::end_effector_pose)
      .def_property_readonly("elbow_position", &RobotPose::elbow_position)
      .def("__mul__",
           [](const RobotPose &robot_pose, const Affine &affine) { return robot_pose * affine; },
           py::is_operator())
      .def("__rmul__",
           [](const RobotPose &robot_pose, const Affine &affine) { return affine * robot_pose; },
           py::is_operator())
      .def("__repr__", robotPoseToStr)
      .def(py::pickle(
          [](const RobotPose &robot_pose) {  // __getstate__
            return py::make_tuple(robot_pose.end_effector_pose(), robot_pose.elbow_position());
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            return RobotPose(t[0].cast<Affine>(), t[1].cast<std::optional<double>>());
          }
      ));
  py::implicitly_convertible<Affine, RobotPose>();

  py::class_<RobotVelocity>(m, "RobotVelocity")
      .def(py::init<Twist, double>(), "end_effector_twist"_a, "elbow_velocity"_a = 0.0)
      .def(py::init<const RobotVelocity &>()) // Copy constructor
      .def("change_end_effector_frame", &RobotVelocity::changeEndEffectorFrame, "offset_world_frame"_a)
      .def_property_readonly("end_effector_twist", &RobotVelocity::end_effector_twist)
      .def_property_readonly("elbow_velocity", &RobotVelocity::elbow_velocity)
      .def("__rmul__",
           [](const RobotVelocity &robot_velocity, const Affine &affine) { return affine * robot_velocity; },
           py::is_operator())
      .def("__rmul__",
           [](const RobotVelocity &robot_velocity, const Eigen::Quaterniond &quaternion) {
             return quaternion * robot_velocity;
           },
           py::is_operator())
      .def("__repr__", robotVelocityToStr)
      .def(py::pickle(
          [](const RobotVelocity &robot_velocity) {  // __getstate__
            return py::make_tuple(robot_velocity.end_effector_twist(), robot_velocity.elbow_velocity());
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            return RobotVelocity(t[0].cast<Twist>(), t[1].cast<double>());
          }
      ));
  py::implicitly_convertible<Twist, RobotVelocity>();

  py::class_<CartesianState>(m, "CartesianState")
      .def(py::init<const RobotPose &>(), "pose"_a)
      .def(py::init<const RobotPose &, const RobotVelocity &>(), "pose"_a, "velocity"_a)
      .def(py::init<const CartesianState &>()) // Copy constructor
      .def("transform_with", &CartesianState::transformWith, "transform"_a)
      .def("change_end_effector_frame", &CartesianState::changeEndEffectorFrame, "transform"_a)
      .def_property_readonly("pose", &CartesianState::pose)
      .def_property_readonly("velocity", &CartesianState::velocity)
      .def("__rmul__",
           [](const CartesianState &cartesian_state, const Affine &affine) { return affine * cartesian_state; },
           py::is_operator())
      .def("__repr__", [](const CartesianState &cartesian_state) {
        std::stringstream ss;
        ss << "CartesianState(pose=" << robotPoseToStr(cartesian_state.pose())
           << ", velocity=" << robotVelocityToStr(cartesian_state.velocity()) << ")";
        return ss.str();
      })
      .def(py::pickle(
          [](const CartesianState &cartesian_state) {  // __getstate__
            return py::make_tuple(cartesian_state.pose(), cartesian_state.velocity());
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            return CartesianState(t[0].cast<RobotPose>(), t[1].cast<RobotVelocity>());
          }
      ));
  py::implicitly_convertible<RobotPose, CartesianState>();
  py::implicitly_convertible<Affine, CartesianState>();

  py::class_<JointState>(m, "JointState")
      .def(py::init<const Vector7d &>(), "position"_a)
      .def(py::init<const Vector7d &, const Vector7d &>(), "position"_a, "velocity"_a)
      .def(py::init<const JointState &>()) // Copy constructor
      .def_property_readonly("position", &JointState::position)
      .def_property_readonly("velocity", &JointState::velocity)
      .def("__repr__", [](const JointState &joint_state) {
        std::stringstream ss;
        ss << "JointState(position=" << vecToStr(joint_state.position())
           << ", velocity=" << vecToStr(joint_state.velocity()) << ")";
        return ss.str();
      })
      .def(py::pickle(
          [](const JointState &joint_state) {  // __getstate__
            return py::make_tuple(joint_state.position(), joint_state.velocity());
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            return JointState(t[0].cast<Vector7d>(), t[1].cast<Vector7d>());
          }
      ));
  py::implicitly_convertible<Vector7d, JointState>();
  py::implicitly_convertible<std::array<double, 7>, JointState>();

  py::class_<Kinematics::NullSpaceHandling>(m, "NullSpaceHandling")
      .def(py::init<size_t, double>(), "joint_index"_a, "value"_a)
      .def_readwrite("joint_index", &Kinematics::NullSpaceHandling::joint_index)
      .def_readwrite("value", &Kinematics::NullSpaceHandling::value);

  py::class_<franka::Torques>(m, "Torques")
      .def_readonly("tau_J", &franka::Torques::tau_J)
      .def(py::pickle(
          [](const franka::Torques &torques) {  // __getstate__
            return py::make_tuple(torques.tau_J);
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 1)
              throw std::runtime_error("Invalid state!");
            return franka::Torques(t[0].cast<std::array<double, 7>>());
          }
      ));

  py::class_<franka::JointVelocities>(m, "JointVelocities")
      .def_readonly("dq", &franka::JointVelocities::dq)
      .def(py::pickle(
          [](const franka::JointVelocities &velocities) {  // __getstate__
            return py::make_tuple(velocities.dq);
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 1)
              throw std::runtime_error("Invalid state!");
            return franka::JointVelocities(t[0].cast<std::array<double, 7>>());
          }
      ));

  py::class_<franka::JointPositions>(m, "JointPositions")
      .def_readonly("q", &franka::JointPositions::q)
      .def(py::pickle(
          [](const franka::JointPositions &positions) {  // __getstate__
            return py::make_tuple(positions.q);
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 1)
              throw std::runtime_error("Invalid state!");
            return franka::JointPositions(t[0].cast<std::array<double, 7>>());
          }
      ));

  py::class_<franka::CartesianVelocities>(m, "CartesianVelocities")
      .def_readonly("O_dP_EE", &franka::CartesianVelocities::O_dP_EE)
      .def(py::pickle(
          [](const franka::CartesianVelocities &velocities) {  // __getstate__
            return py::make_tuple(velocities.O_dP_EE, velocities.elbow);
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            return franka::CartesianVelocities(t[0].cast<std::array<double, 6>>(), t[1].cast<std::array<double, 2>>());
          }
      ));

  py::class_<franka::CartesianPose>(m, "CartesianPose")
      .def_readonly("O_T_EE", &franka::CartesianPose::O_T_EE)
      .def(py::pickle(
          [](const franka::CartesianPose &pose) {  // __getstate__
            return py::make_tuple(pose.O_T_EE, pose.elbow);
          },
          [](const py::tuple &t) {  // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            return franka::CartesianPose(t[0].cast<std::array<double, 16>>(), t[1].cast<std::array<double, 2>>());
          }
      ));

  mkMotionAndReactionClasses<franka::Torques>(m, "Torque");
  mkMotionAndReactionClasses<franka::JointVelocities>(m, "JointVelocity");
  mkMotionAndReactionClasses<franka::JointPositions>(m, "JointPosition");
  mkMotionAndReactionClasses<franka::CartesianVelocities>(m, "CartesianVelocity");
  mkMotionAndReactionClasses<franka::CartesianPose>(m, "CartesianPose");

  py::class_<ImpedanceMotion, Motion<franka::Torques>, std::shared_ptr<ImpedanceMotion>>(m, "ImpedanceMotion");

  py::class_<ExponentialImpedanceMotion, ImpedanceMotion, std::shared_ptr<ExponentialImpedanceMotion>>(
      m, "ExponentialImpedanceMotion")
      .def(py::init<>([](
               const Affine &target, ReferenceType target_type, double translational_stiffness,
               double rotational_stiffness, std::optional<std::array<std::optional<double>, 6>> force_constraints,
               double exponential_decay = 0.005) {
             Eigen::Vector<bool, 6> force_constraints_active = Eigen::Vector<bool, 6>::Zero();
             Eigen::Vector<double, 6> force_constraints_value;
             if (force_constraints.has_value()) {
               for (int i = 0; i < 6; i++) {
                 force_constraints_value[i] = force_constraints.value()[i].value_or(NAN);
                 force_constraints_active[i] = force_constraints.value()[i].has_value();
               }
             }
             return std::make_shared<ExponentialImpedanceMotion>(
                 target,
                 ExponentialImpedanceMotion::Params{
                     target_type, translational_stiffness, rotational_stiffness, force_constraints_value,
                     force_constraints_active, exponential_decay});
           }),
           "target"_a,
           py::arg_v("target_type", ReferenceType::Absolute, "_franky.ReferenceType.Absolute"),
           "translational_stiffness"_a = 2000,
           "rotational_stiffness"_a = 200,
           "force_constraints"_a = std::nullopt,
           "exponential_decay"_a = 0.005);

  py::class_<CartesianImpedanceMotion, ImpedanceMotion, std::shared_ptr<CartesianImpedanceMotion>>(
      m, "CartesianImpedanceMotion")
      .def(py::init<>([](
               const Affine &target,
               std::chrono::duration<double> duration,
               ReferenceType target_type,
               double translational_stiffness,
               double rotational_stiffness,
               std::optional<std::array<std::optional<double>, 6>> force_constraints,
               bool return_when_finished,
               double finish_wait_factor) {
             Eigen::Vector<bool, 6> force_constraints_active = Eigen::Vector<bool, 6>::Zero();
             Eigen::Vector<double, 6> force_constraints_value;
             if (force_constraints.has_value()) {
               for (int i = 0; i < 6; i++) {
                 force_constraints_value[i] = force_constraints.value()[i].value_or(NAN);
                 force_constraints_active[i] = force_constraints.value()[i].has_value();
               }
             }
             return std::make_shared<CartesianImpedanceMotion>(
                 target, duration,
                 CartesianImpedanceMotion::Params{
                     target_type, translational_stiffness, rotational_stiffness, force_constraints_value,
                     force_constraints_active, return_when_finished, finish_wait_factor});
           }),
           "target"_a,
           "duration"_a,
           py::arg_v("target_type", ReferenceType::Absolute, "_franky.ReferenceType.Absolute"),
           "translational_stiffness"_a = 2000,
           "rotational_stiffness"_a = 200,
           "force_constraints"_a = std::nullopt,
           "return_when_finished"_a = true,
           "finish_wait_factor"_a = 1.2);

  py::class_<PositionWaypoint<JointState>>(m, "JointWaypoint")
      .def(py::init<>(
               [](
                   const JointState &target,
                   ReferenceType reference_type,
                   RelativeDynamicsFactor relative_dynamics_factor,
                   std::optional<double> minimum_time,
                   std::chrono::duration<double> hold_target_duration
               ) {
                 return PositionWaypoint<JointState>{
                     {target, relative_dynamics_factor, minimum_time}, reference_type
                 };
               }
           ),
           "target"_a,
           py::arg_v("reference_type", ReferenceType::Absolute, "_franky.ReferenceType.Absolute"),
           "relative_dynamics_factor"_a = 1.0,
           "minimum_time"_a = std::nullopt,
           "hold_target_duration"_a = std::chrono::duration<double>(0.0))
      .def_readonly("target", &PositionWaypoint<JointState>::target)
      .def_readonly("reference_type", &PositionWaypoint<JointState>::reference_type)
      .def_readonly("relative_dynamics_factor", &PositionWaypoint<JointState>::relative_dynamics_factor)
      .def_readonly("minimum_time", &PositionWaypoint<JointState>::minimum_time)
      .def_readonly("hold_target_duration", &PositionWaypoint<JointState>::hold_target_duration);

  py::class_<JointWaypointMotion, Motion<franka::JointPositions>, std::shared_ptr<JointWaypointMotion>>(
      m, "JointWaypointMotion")
      .def(py::init<>([](
               const std::vector<PositionWaypoint<JointState>> &waypoints,
               RelativeDynamicsFactor relative_dynamics_factor,
               bool return_when_finished) {
             return std::make_shared<JointWaypointMotion>(
                 waypoints, relative_dynamics_factor, return_when_finished);
           }),
           "waypoints"_a,
           "relative_dynamics_factor"_a = 1.0,
           "return_when_finished"_a = true);

  py::class_<JointMotion, JointWaypointMotion, std::shared_ptr<JointMotion>>(m, "JointMotion")
      .def(py::init<const JointState &, ReferenceType, double, bool>(),
           "target"_a,
           py::arg_v("reference_type", ReferenceType::Absolute, "_franky.ReferenceType.Absolute"),
           "relative_dynamics_factor"_a = 1.0,
           "return_when_finished"_a = true);

  py::class_<PositionWaypoint<CartesianState>>(m, "CartesianWaypoint")
      .def(py::init<>(
               [](
                   const CartesianState &target,
                   ReferenceType reference_type,
                   RelativeDynamicsFactor relative_dynamics_factor,
                   std::optional<double> minimum_time,
                   std::chrono::duration<double> hold_target_duration
               ) {
                 return PositionWaypoint<CartesianState>{
                     {target, relative_dynamics_factor, minimum_time}, reference_type};
               }
           ),
           "target"_a,
           py::arg_v("reference_type", ReferenceType::Absolute, "_franky.ReferenceType.Absolute"),
           "relative_dynamics_factor"_a = 1.0,
           "minimum_time"_a = std::nullopt,
           "hold_target_duration"_a = std::chrono::duration<double>(0.0))
      .def_readonly("target", &PositionWaypoint<CartesianState>::target)
      .def_readonly("reference_type", &PositionWaypoint<CartesianState>::reference_type)
      .def_readonly("relative_dynamics_factor", &PositionWaypoint<CartesianState>::relative_dynamics_factor)
      .def_readonly("minimum_time", &PositionWaypoint<CartesianState>::minimum_time)
      .def_readonly("hold_target_duration", &PositionWaypoint<CartesianState>::hold_target_duration);

  py::class_<CartesianWaypointMotion, Motion<franka::CartesianPose>, std::shared_ptr<CartesianWaypointMotion>>(
      m, "CartesianWaypointMotion")
      .def(py::init<>([](
               const std::vector<PositionWaypoint<CartesianState>> &waypoints,
               const std::optional<Affine> &ee_frame = std::nullopt,
               RelativeDynamicsFactor relative_dynamics_factor = 1.0,
               bool return_when_finished = true) {
             return std::make_shared<CartesianWaypointMotion>(
                 waypoints, relative_dynamics_factor, return_when_finished, ee_frame.value_or(Affine::Identity()));
           }),
           "waypoints"_a,
           "ee_frame"_a = std::nullopt,
           "relative_dynamics_factor"_a = 1.0,
           "return_when_finished"_a = true);

  py::class_<CartesianMotion, CartesianWaypointMotion, std::shared_ptr<CartesianMotion>>(m, "CartesianMotion")
      .def(py::init<>([](
               const CartesianState &target,
               ReferenceType reference_type,
               RelativeDynamicsFactor relative_dynamics_factor,
               bool return_when_finished,
               const std::optional<Affine> &ee_frame) {
             return std::make_shared<CartesianMotion>(
                 target,
                 reference_type,
                 relative_dynamics_factor,
                 return_when_finished,
                 ee_frame.value_or(Affine::Identity()));
           }),
           "target"_a,
           py::arg_v("reference_type", ReferenceType::Absolute, "_franky.ReferenceType.Absolute"),
           "relative_dynamics_factor"_a = 1.0,
           "return_when_finished"_a = true,
           "ee_frame"_a = std::nullopt);

  // Backwards compatibility
  m.attr("LinearMotion") = m.attr("CartesianMotion");

  py::class_<StopMotion<franka::CartesianPose>,
             Motion<franka::CartesianPose>,
             std::shared_ptr<StopMotion<franka::CartesianPose>>>(m, "CartesianPoseStopMotion")
      .def(py::init<>());

  py::class_<StopMotion<franka::JointPositions>,
             Motion<franka::JointPositions>,
             std::shared_ptr<StopMotion<franka::JointPositions>>>(m, "JointPositionStopMotion")
      .def(py::init<>());

  py::class_<Gripper>(m, "Gripper")
      .def(py::init<const std::string &>(), "fci_hostname"_a)
      .def("grasp", &Gripper::grasp,
           "width"_a,
           "speed"_a,
           "force"_a,
           "epsilon_inner"_a = 0.005,
           "epsilon_outer"_a = 0.005,
           py::call_guard<py::gil_scoped_release>())
      .def("grasp_async", &Gripper::graspAsync,
           "width"_a,
           "speed"_a,
           "force"_a,
           "epsilon_inner"_a = 0.005,
           "epsilon_outer"_a = 0.005,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &Gripper::move, "width"_a, "speed"_a, py::call_guard<py::gil_scoped_release>())
      .def("move_async", &Gripper::moveAsync, "width"_a, "speed"_a, py::call_guard<py::gil_scoped_release>())
      .def("open", &Gripper::open, "speed"_a, py::call_guard<py::gil_scoped_release>())
      .def("open_async", &Gripper::openAsync, "speed"_a, py::call_guard<py::gil_scoped_release>())
      .def("homing", &Gripper::homing, py::call_guard<py::gil_scoped_release>())
      .def("homing_async", &Gripper::homingAsync, py::call_guard<py::gil_scoped_release>())
      .def("stop", &Gripper::stop, py::call_guard<py::gil_scoped_release>())
      .def("stop_async", &Gripper::stopAsync, py::call_guard<py::gil_scoped_release>())
      .def_property_readonly("state", &Gripper::state)
      .def_property_readonly("server_version", (uint16_t (Gripper::*)()) &Gripper::serverVersion)
      .def_property_readonly("width", &Gripper::width)
      .def_property_readonly("is_grasped", &Gripper::is_grasped)
      .def_property_readonly("max_width", &Gripper::max_width);

  py::class_<Kinematics>(m, "Kinematics")
      .def_static("forward", &Kinematics::forward, "q"_a)
      .def_static("forward_elbow", &Kinematics::forwardElbow, "q"_a)
      .def_static("forward_euler", &Kinematics::forwardEuler, "q"_a)
      .def_static("jacobian", &Kinematics::jacobian, "q"_a);

  py::class_<Robot>(m, "RobotInternal")
      .def(py::init<>([](
               const std::string &fci_hostname,
               RelativeDynamicsFactor relative_dynamics_factor,
               double default_torque_threshold,
               double default_force_threshold,
               franka::ControllerMode controller_mode,
               franka::RealtimeConfig realtime_config) {
             return std::make_unique<Robot>(
                 fci_hostname, Robot::Params{
                     relative_dynamics_factor, default_torque_threshold, default_force_threshold,
                     controller_mode, realtime_config});
           }),
           "fci_hostname"_a,
           "relative_dynamics_factor"_a = 1.0,
           "default_torque_threshold"_a = 20.0,
           "default_force_threshold"_a = 30.0,
           py::arg_v("controller_mode",
                     franka::ControllerMode::kJointImpedance,
                     "_franky.ControllerMode.JointImpedance"),
           py::arg_v("realtime_config", franka::RealtimeConfig::kEnforce, "_franky.RealtimeConfig.Enforce"))
      .def("recover_from_errors", &Robot::recoverFromErrors)
      .def("move", &robotMove<franka::CartesianPose>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &robotMove<franka::CartesianVelocities>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &robotMove<franka::JointPositions>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &robotMove<franka::JointVelocities>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("move", &robotMove<franka::Torques>, "motion"_a, "asynchronous"_a = false,
           py::call_guard<py::gil_scoped_release>())
      .def("join_motion", [](Robot &robot, std::optional<double> timeout) {
        if (timeout.has_value())
          return robot.joinMotion<double, std::ratio<1>>(std::chrono::duration<double, std::ratio<1>>(timeout.value()));
        return robot.joinMotion();
      }, "timeout"_a = std::nullopt, py::call_guard<py::gil_scoped_release>())
      .def("poll_motion", &Robot::pollMotion)
      .def("set_collision_behavior",
           py::overload_cast<
               const Robot::ScalarOrArray<7> &,
               const Robot::ScalarOrArray<6> &
           >(&Robot::setCollisionBehavior),
           "torque_thresholds"_a, "force_thresholds"_a)
      .def("set_collision_behavior",
           py::overload_cast<
               const Robot::ScalarOrArray<7> &,
               const Robot::ScalarOrArray<7> &,
               const Robot::ScalarOrArray<6> &,
               const Robot::ScalarOrArray<6> &
           >(&Robot::setCollisionBehavior),
           "lower_torque_threshold"_a,
           "upper_torque_threshold"_a,
           "lower_force_threshold"_a,
           "upper_force_threshold"_a)
      .def("set_collision_behavior",
           py::overload_cast<
               const Robot::ScalarOrArray<7> &,
               const Robot::ScalarOrArray<7> &,
               const Robot::ScalarOrArray<7> &,
               const Robot::ScalarOrArray<7> &,
               const Robot::ScalarOrArray<6> &,
               const Robot::ScalarOrArray<6> &,
               const Robot::ScalarOrArray<6> &,
               const Robot::ScalarOrArray<6> &
           >(&Robot::setCollisionBehavior),
           "lower_torque_threshold_acceleration"_a,
           "upper_torque_threshold_acceleration"_a,
           "lower_torque_threshold_nominal"_a,
           "upper_torque_threshold_nominal"_a,
           "lower_force_threshold_acceleration"_a,
           "upper_force_threshold_acceleration"_a,
           "lower_force_threshold_nominal"_a,
           "upper_force_threshold_nominal"_a)
      .def("set_joint_impedance", &Robot::setJointImpedance, "K_theta"_a)
      .def("set_cartesian_impedance", &Robot::setCartesianImpedance, "K_x"_a)
      .def("set_guiding_mode", &Robot::setGuidingMode, "guiding_mode"_a, "elbow"_a)
      .def("set_k", &Robot::setK, "EE_T_K"_a)
      .def("set_ee", &Robot::setEE, "NE_T_EE"_a)
      .def("set_load", &Robot::setLoad, "load_mass"_a, "F_x_Cload"_a, "load_inertia"_a)
      .def("stop", &Robot::stop)
      .def_property("relative_dynamics_factor", &Robot::relative_dynamics_factor, &Robot::setRelativeDynamicsFactor)
      .def_property_readonly("has_errors", &Robot::hasErrors)
      .def_property_readonly("current_pose", &Robot::currentPose)
      .def_property_readonly("current_cartesian_velocity", &Robot::currentCartesianVelocity)
      .def_property_readonly("current_cartesian_state", &Robot::currentCartesianState)
      .def_property_readonly("current_joint_state", &Robot::currentJointState)
      .def_property_readonly("current_joint_velocities", &Robot::currentJointVelocities)
      .def_property_readonly("current_joint_positions", &Robot::currentJointPositions)
      .def_property_readonly("state", &Robot::state)
      .def_property_readonly("is_in_control", &Robot::is_in_control)
      .def_property_readonly("fci_hostname", &Robot::fci_hostname)
      .def_property_readonly("current_control_signal_type", &Robot::current_control_signal_type)
      .def_readonly_static("max_translation_velocity", &Robot::max_translation_velocity, "[m/s]")
      .def_readonly_static("max_rotation_velocity", &Robot::max_rotation_velocity, "[rad/s]")
      .def_readonly_static("max_elbow_velocity", &Robot::max_elbow_velocity, "[rad/s]")
      .def_readonly_static("max_translation_acceleration", &Robot::max_translation_acceleration, "[m/s²]")
      .def_readonly_static("max_rotation_acceleration", &Robot::max_rotation_acceleration, "[rad/s²]")
      .def_readonly_static("max_elbow_acceleration", &Robot::max_elbow_acceleration, "[rad/s²]")
      .def_readonly_static("max_translation_jerk", &Robot::max_translation_jerk, "[m/s³]")
      .def_readonly_static("max_rotation_jerk", &Robot::max_rotation_jerk, "[rad/s³]")
      .def_readonly_static("max_elbow_jerk", &Robot::max_elbow_jerk, "[rad/s³]")
      .def_readonly_static("degrees_of_freedom", &Robot::degrees_of_freedoms)
      .def_readonly_static("control_rate", &Robot::control_rate, "[s]")
      .def_property_readonly_static("max_joint_velocity", [](py::object) {
        return Vector7d::Map(Robot::max_joint_velocity.data());
      }, "[rad/s]")
      .def_property_readonly_static("max_joint_acceleration", [](py::object) {
        return Vector7d::Map(Robot::max_joint_acceleration.data());
      }, "[rad/s²]")
      .def_property_readonly_static("max_joint_jerk", [](py::object) {
        return Vector7d::Map(Robot::max_joint_jerk.data());
      }, "[rad/s^3]")
      .def_static("forward_kinematics", &Robot::forwardKinematics, "q"_a);
  // .def_static("inverse_kinematics", &Robot::inverseKinematics, "target"_a, "q0"_a);

  py::class_<std::shared_future<bool>>(m, "BoolFuture")
      .def("wait", [](const std::shared_future<bool> &future, std::optional<double> timeout) {
        if (timeout.has_value())
          return future.wait_for(std::chrono::duration<double>(timeout.value())) == std::future_status::ready;
        future.wait();
        return true;
      }, "timeout"_a = std::nullopt, py::call_guard<py::gil_scoped_release>())
      .def("get", &std::shared_future<bool>::get, py::call_guard<py::gil_scoped_release>());

  py::register_exception<franka::Exception>(m, "Exception");
  py::register_exception<franka::CommandException>(m, "CommandException");
  py::register_exception<franka::ControlException>(m, "ControlException");
  py::register_exception<franka::IncompatibleVersionException>(m, "IncompatibleVersionException");
  py::register_exception<franka::InvalidOperationException>(m, "InvalidOperationException");
  py::register_exception<franka::ModelException>(m, "ModelException");
  py::register_exception<franka::NetworkException>(m, "NetworkException");
  py::register_exception<franka::ProtocolException>(m, "ProtocolException");
  py::register_exception<franka::RealtimeException>(m, "RealtimeException");
  py::register_exception<franky::InvalidMotionTypeException>(m, "InvalidMotionTypeException");
  py::register_exception<franky::ReactionRecursionException>(m, "ReactionRecursionException");
  py::register_exception<franky::GripperException>(m, "GripperException");
}
