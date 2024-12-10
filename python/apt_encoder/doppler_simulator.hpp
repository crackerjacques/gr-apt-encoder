#include <pybind11/pybind11.h>
#include <gnuradio/apt_encoder/doppler_simulator.hpp>

namespace py = pybind11;

void bind_doppler_simulator(py::module& m) {
    using doppler_simulator = gr::apt_encoder::doppler_simulator;
    
    py::class_<doppler_simulator, gr::sync_block, std::shared_ptr<doppler_simulator>>(m, "doppler_simulator")
        .def(py::init(&doppler_simulator::make),
             py::arg("sample_rate"),
             py::arg("center_freq"),
             py::arg("range_km"),
             py::arg("sat_velocity") = 7.8,
             py::arg("auto_trim") = true,
             py::arg("trim_threshold") = 20.0)
        .def("set_range", &doppler_simulator::set_range)
        .def("set_velocity", &doppler_simulator::set_velocity)
        .def("get_doppler_shift", &doppler_simulator::get_doppler_shift)
        .def("get_signal_strength", &doppler_simulator::get_signal_strength)
        .def("set_auto_trim", &doppler_simulator::set_auto_trim);
}