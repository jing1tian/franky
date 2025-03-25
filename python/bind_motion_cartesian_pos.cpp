#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include "franky.hpp"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace franky;

void bind_motion_cartesian_pos(py::module &m) {
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
               const RelativeDynamicsFactor &relative_dynamics_factor = 1.0,
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

  py::class_<StopMotion<franka::CartesianPose>,
             Motion<franka::CartesianPose>,
             std::shared_ptr<StopMotion<franka::CartesianPose>>>(m, "CartesianPoseStopMotion")
      .def(py::init<>());
}