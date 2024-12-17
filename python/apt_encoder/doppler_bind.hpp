#include <pybind11/pybind11.h>
#include <gnuradio/apt_encoder/doppler_simulator.hpp>

namespace py = pybind11;

void bind_doppler_simulator(py::module& m) {
    using doppler_simulator = gr::apt_encoder::doppler_simulator;
    
    py::class_<doppler_simulator, gr::sync_block, std::shared_ptr<doppler_simulator>>(
        m, "doppler_simulator")
        .def_static("make", 
            &doppler_simulator::make,
            py::arg("sample_rate"),
            py::arg("center_freq"),
            py::arg("range_km"),
            py::arg("sat_velocity") = 7.8f,
            py::arg("auto_trim") = true,
            py::arg("trim_threshold") = 20.0f,
            py::arg("antenna_lat") = 35.0f,
            py::arg("antenna_lon") = 135.0f,
            py::arg("antenna_alt") = 0.0f,
            py::arg("start_lat") = 0.0f,
            py::arg("start_lon") = 0.0f,
            py::arg("orbital_inclination") = 98.7f,
            py::arg("ascending") = true)

        .def("set_range", &doppler_simulator::set_range,
            py::arg("range_km"))
        .def("set_velocity", &doppler_simulator::set_velocity,
            py::arg("velocity"))
        .def("set_antenna_position", &doppler_simulator::set_antenna_position,
            py::arg("lat"),
            py::arg("lon"),
            py::arg("alt"))
        .def("set_auto_trim", &doppler_simulator::set_auto_trim,
            py::arg("enable"))
        .def("set_antenna_type", &doppler_simulator::set_antenna_type,
            py::arg("type"))
        .def("set_antenna_orientation", &doppler_simulator::set_antenna_orientation,
            py::arg("azimuth"),
            py::arg("elevation"))
        .def("get_antenna_type", &doppler_simulator::get_antenna_type)
        .def("get_doppler_shift", &doppler_simulator::get_doppler_shift)
        .def("get_signal_strength", &doppler_simulator::get_signal_strength)
        .def("get_elevation", &doppler_simulator::get_elevation)
        .def("get_azimuth", &doppler_simulator::get_azimuth)
        .def("get_current_lat", &doppler_simulator::get_current_lat)
        .def("get_current_lon", &doppler_simulator::get_current_lon)
        .def("get_current_heading", &doppler_simulator::get_current_heading)
        .def("get_orbital_period", &doppler_simulator::get_orbital_period)
        .def("get_current_distance", &doppler_simulator::get_current_distance)
        .def("get_current_velocity", &doppler_simulator::get_current_velocity);
}