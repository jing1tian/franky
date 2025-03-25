#include <pybind11/pybind11.h>
#include <Python.h>

#include <franka/robot_state.h>

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal

void bind_enums(py::module &);
void bind_errors(py::module &);
void bind_kinematics(py::module &);
void bind_misc(py::module &);
void bind_motion_cartesian_pos(py::module &);
void bind_motion_join_pos(py::module &);
void bind_motion_join_vel(py::module &);
void bind_reactions(py::module &);
void bind_robot(py::module &);
void bind_robot_state(py::module &);
void bind_state_repr(py::module &);

PYBIND11_MODULE(_franky, m) {
  m.doc() = "High-Level Control Library for Franka Robots";

  bind_enums(m);
  bind_errors(m);
  bind_kinematics(m);
  bind_misc(m);
  bind_motion_cartesian_pos(m);
  bind_motion_join_pos(m);
  bind_motion_join_vel(m);
  bind_reactions(m);
  bind_robot(m);
  bind_robot_state(m);
  bind_state_repr(m);
}
