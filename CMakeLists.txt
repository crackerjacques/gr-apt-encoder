# root dir
cmake_minimum_required(VERSION 3.8)
project(gr-apt-encoder CXX C)

find_package(Gnuradio "3.9" REQUIRED)
include(GrVersion)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets Svg REQUIRED)

#find_package(ALSA REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Development)
find_package(pybind11 REQUIRED)
find_package(OpenCV REQUIRED)

include(GrPlatform)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to Release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_STANDARD 14)

include_directories(
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/include
    ${Gnuradio_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Svg_INCLUDE_DIRS}
)


add_subdirectory(lib)
add_subdirectory(include/gnuradio/apt_encoder)
add_subdirectory(grc)
add_subdirectory(python)

