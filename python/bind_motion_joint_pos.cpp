#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include "franky.hpp"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace franky;

void bind_motion_joint_pos(py::module &m) {
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

  py::class_<StopMotion<franka::JointPositions>,
             Motion<franka::JointPositions>,
             std::shared_ptr<StopMotion<franka::JointPositions>>>(m, "JointPositionStopMotion")
      .def(py::init<>());
}