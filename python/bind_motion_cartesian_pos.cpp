#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "franky.hpp"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace franky;

void bind_motion_cartesian_pos(py::module &m) {
  py::class_<PositionWaypoint<CartesianState>>(m, "CartesianWaypoint")
      .def(py::init<>(
               [](
                   const CartesianState &target,
                   ReferenceType reference_type,
                   RelativeDynamicsFactor relative_dynamics_factor,
                   std::optional<double> minimum_time,
                   franka::Duration hold_target_duration
               ) {
                 return PositionWaypoint<CartesianState>{
                     {target, relative_dynamics_factor, minimum_time, hold_target_duration}, reference_type};
               }
           ),
           "target"_a,
           py::arg_v("reference_type", ReferenceType::kAbsolute, "_franky.ReferenceType.Absolute"),
           "relative_dynamics_factor"_a = 1.0,
           "minimum_time"_a = std::nullopt,
           "hold_target_duration"_a = franka::Duration(0))
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
           py::arg_v("reference_type", ReferenceType::kAbsolute, "_franky.ReferenceType.Absolute"),
           "relative_dynamics_factor"_a = 1.0,
           "return_when_finished"_a = true,
           "ee_frame"_a = std::nullopt);

  py::class_<StopMotion<franka::CartesianPose>,
             Motion<franka::CartesianPose>,
             std::shared_ptr<StopMotion<franka::CartesianPose>>>(m, "CartesianPoseStopMotion")
      .def(py::init<>());
}