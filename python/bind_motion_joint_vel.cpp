#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include "franky.hpp"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace franky;

void bind_motion_joint_vel(py::module &m) {
  py::class_<VelocityWaypoint<Vector7d>>(m, "JointWaypoint")
      .def(py::init<>(
               [](
                   const Vector7d &target,
                   RelativeDynamicsFactor relative_dynamics_factor,
                   std::optional<double> minimum_time,
                   std::chrono::duration<double> hold_target_duration
               ) {
                 return VelocityWaypoint<Vector7d>{
                     target, relative_dynamics_factor, minimum_time
                 };
               }
           ),
           "target"_a,
           "relative_dynamics_factor"_a = 1.0,
           "minimum_time"_a = std::nullopt,
           "hold_target_duration"_a = std::chrono::duration<double>(0.0))
      .def_readonly("target", &VelocityWaypoint<Vector7d>::target)
      .def_readonly("relative_dynamics_factor", &VelocityWaypoint<Vector7d>::relative_dynamics_factor)
      .def_readonly("minimum_time", &VelocityWaypoint<Vector7d>::minimum_time)
      .def_readonly("hold_target_duration", &VelocityWaypoint<Vector7d>::hold_target_duration);

  py::class_<JointVelocityWaypointMotion, Motion<franka::JointVelocities>, std::shared_ptr<JointVelocityWaypointMotion>>(
      m, "JointVelocityWaypointMotion")
      .def(py::init<>([](
               const std::vector<VelocityWaypoint<Vector7d>> &waypoints,
               RelativeDynamicsFactor relative_dynamics_factor) {
             return std::make_shared<JointVelocityWaypointMotion>(
                 waypoints, relative_dynamics_factor);
           }),
           "waypoints"_a,
           "relative_dynamics_factor"_a = 1.0);

  py::class_<JointMotion, JointVelocityWaypointMotion, std::shared_ptr<JointMotion>>(m, "JointMotion")
      .def(py::init<const Vector7d &, ReferenceType, double, bool>(),
           "target"_a,
           py::arg_v("reference_type", ReferenceType::Absolute, "_franky.ReferenceType.Absolute"),
           "relative_dynamics_factor"_a = 1.0,
           "return_when_finished"_a = true);

  py::class_<StopMotion<franka::JointVelocities>,
             Motion<franka::JointVelocities>,
             std::shared_ptr<StopMotion<franka::JointVelocities>>>(m, "JointVelocityStopMotion")
      .def(py::init<>());
}