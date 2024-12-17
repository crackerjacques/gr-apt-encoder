# python/
include(GrPython)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
COPYONLY)

# Python files to install
install(
    FILES
        apt_encoder/position_visualizer.py
        apt_encoder/apt_encoder_signal_strength_plot.py
        apt_encoder/constants.py
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/apt_encoder
)

# Install SVG map file to multiple locations for better discovery
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/apt_encoder/world_map.svg
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/apt_encoder
)

install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/apt_encoder/world_map.svg
    DESTINATION share/gnuradio/apt_encoder
)


# Add Python bindings library
add_library(encoder_python MODULE
    apt_encoder/encoder_python.cpp
)

target_include_directories(encoder_python
    PRIVATE
        ${PYTHON_INCLUDE_DIRS}
        ${PYBIND11_INCLUDE_DIR}
)

target_link_libraries(encoder_python
    PRIVATE
        gnuradio-apt-encoder
        Python::Module
        pybind11::module
)

set_target_properties(encoder_python PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python/gnuradio/apt_encoder
)

install(TARGETS encoder_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/apt_encoder
)

GR_PYTHON_INSTALL(
    FILES
        __init__.py
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/apt_encoder
)