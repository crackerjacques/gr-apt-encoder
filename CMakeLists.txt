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

find_library(LIBSGP4_LIBRARY
    NAMES sgp4 sgp4s
    PATHS /usr/local/lib
    REQUIRED
)

set(LIBSGP4_INCLUDE_DIR /usr/local/include/libsgp4)

if(NOT LIBSGP4_LIBRARY)
    message(FATAL_ERROR "libsgp4 library not found")
endif()

if(NOT IS_DIRECTORY ${LIBSGP4_INCLUDE_DIR})
    message(FATAL_ERROR "libsgp4 include directory not found")
endif()

message(STATUS "Found libsgp4: ${LIBSGP4_LIBRARY}")
message(STATUS "Found libsgp4 include: ${LIBSGP4_INCLUDE_DIR}")

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
    ${LIBSGP4_INCLUDE_DIR}
)

enable_testing()


add_subdirectory(lib)
add_subdirectory(include/gnuradio/apt_encoder)
add_subdirectory(grc)
add_subdirectory(python)
add_subdirectory(test)
