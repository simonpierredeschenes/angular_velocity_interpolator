#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "angular_velocity_interpolator.h"

namespace py = pybind11;

PYBIND11_MODULE(pyangular_velocity_interpolator, module_handle)
{
    module_handle.def("interpolate_angular_velocities", &interpolateAngularVelocities, py::arg("measurement_stamps"), py::arg("velocity_stamps"), py::arg("angular_velocities"), py::arg("information_matrices"), py::arg("qc_diag"));
}
