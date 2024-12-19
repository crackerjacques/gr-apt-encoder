# lib/CMakeLists.txt

add_library(gnuradio-apt-encoder SHARED
    encoder_impl.cpp
    image_handler.cpp
    apt_generator.cpp
    apt_format.cpp
    doppler_simulator_impl.cpp
    antenna_pattern.cpp
    sgp4_orbit.cpp        # SGP4関連のソースを追加
    orbit_utils.cpp
    doppler_simulator_sgp4_impl.cpp
)

target_include_directories(gnuradio-apt-encoder
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${LIBSGP4_INCLUDE_DIR}
)

target_link_libraries(gnuradio-apt-encoder
    PUBLIC
        gnuradio::gnuradio-runtime
    PRIVATE
        Threads::Threads
        ${LIBSGP4_LIBRARY}
)

install(TARGETS gnuradio-apt-encoder
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

install(FILES
    apt_format.hpp
    apt_generator.hpp
    apt_timing.hpp
    image_handler.hpp
    antenna_pattern.hpp
    sgp4_orbit.hpp
    orbit_utils.hpp
    DESTINATION include/gnuradio/apt_encoder/impl
)