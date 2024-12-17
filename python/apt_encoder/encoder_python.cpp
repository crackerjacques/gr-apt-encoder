#include <pybind11/pybind11.h>
#include <gnuradio/apt_encoder/doppler_simulator.hpp>
#include <gnuradio/apt_encoder/encoder.hpp>

namespace py = pybind11;

void bind_antenna_type(py::module& m) {
    py::enum_<gr::apt_encoder::AntennaType>(m, "AntennaType")
        .value("OMNI", gr::apt_encoder::AntennaType::OMNI)
        .value("DIPOLE", gr::apt_encoder::AntennaType::DIPOLE)
        .value("DOUBLE_CROSS", gr::apt_encoder::AntennaType::DOUBLE_CROSS)
        .value("QFH", gr::apt_encoder::AntennaType::QFH)
        .value("TURNSTILE", gr::apt_encoder::AntennaType::TURNSTILE)
        .export_values();
}

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

void bind_encoder(py::module& m) {
    using encoder = gr::apt_encoder::encoder;
    
    py::class_<encoder, gr::sync_block, std::shared_ptr<encoder>>(
        m, "encoder")
        .def_static("make",
            &encoder::make,
            py::arg("image_file"),
            py::arg("second_file") = "",
            py::arg("sample_rate") = 24000.0,
            py::arg("carrier_frequency") = 2400.0,
            py::arg("loop") = false,
            py::arg("mode") = 'N')
        .def("estimate_duration", &encoder::estimate_duration)
        .def("estimate_samples", &encoder::estimate_samples)
        .def("calculate_path_length", &encoder::calculate_path_length)
        .def("reset", &encoder::reset);
}

PYBIND11_MODULE(encoder_python, m) {
    m.doc() = "GNU Radio APT Encoder Python bindings";
    
    try {
        bind_antenna_type(m);
        bind_doppler_simulator(m);
        bind_encoder(m);
        
        m.attr("__version__") = "1.1.0";
    } catch (const std::exception& e) {
        throw py::error_already_set();
    }
}