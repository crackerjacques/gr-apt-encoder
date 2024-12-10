#include <pybind11/pybind11.h>
#include <gnuradio/apt_encoder/encoder.hpp>
#include <gnuradio/apt_encoder/doppler_simulator.hpp>

namespace py = pybind11;

PYBIND11_MODULE(encoder_python, m) {
    m.doc() = "APT Encoder and Doppler simulator blocks";

    // Encoder class
    py::class_<gr::apt_encoder::encoder,
               gr::sync_block,
               std::shared_ptr<gr::apt_encoder::encoder>>
        (m, "encoder")
        .def_static("make",
            &gr::apt_encoder::encoder::make,
            py::arg("image_file"),
            py::arg("second_file") = "",
            py::arg("sample_rate") = 24000,
            py::arg("carrier_frequency") = 2400,
            py::arg("repeat_mode") = false,
            py::arg("mode") = 'N');

    // Doppler simulator class
    py::class_<gr::apt_encoder::doppler_simulator,
               gr::sync_block,
               std::shared_ptr<gr::apt_encoder::doppler_simulator>>
        (m, "doppler_simulator")
        .def_static("make",
            &gr::apt_encoder::doppler_simulator::make,
            py::arg("sample_rate"),
            py::arg("center_freq"),
            py::arg("range_km"),
            py::arg("sat_velocity") = 7.8f,
            py::arg("auto_trim") = true,
            py::arg("trim_threshold") = 20.0f)
        .def("set_range", &gr::apt_encoder::doppler_simulator::set_range)
        .def("set_velocity", &gr::apt_encoder::doppler_simulator::set_velocity)
        .def("get_doppler_shift", &gr::apt_encoder::doppler_simulator::get_doppler_shift)
        .def("get_signal_strength", &gr::apt_encoder::doppler_simulator::get_signal_strength)
        .def("set_auto_trim", &gr::apt_encoder::doppler_simulator::set_auto_trim);
}