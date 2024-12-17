#ifndef INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_IMPL_H
#define INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_IMPL_H

#include <gnuradio/apt_encoder/doppler_simulator.hpp>
#include "antenna_pattern.hpp"
#include <cmath>

namespace gr {
namespace apt_encoder {

class SatelliteDistanceCalculator {
private:
    static constexpr double EARTH_RADIUS = 6371.0;    // km
    static constexpr double DEG_TO_RAD = M_PI / 180.0;

public:
    static double calculateDistance(
        double ant_lat, double ant_lon, double ant_alt,
        double sat_lat, double sat_lon, double sat_alt
    );
    
    static double calculateElevation(
        double ant_lat, double ant_lon, double ant_alt,
        double sat_lat, double sat_lon, double sat_alt
    );
    
    static double calculateAzimuth(
        double ant_lat, double ant_lon,
        double sat_lat, double sat_lon
    );
    
    static bool isVisible(
        double ant_lat, double ant_lon, double ant_alt,
        double sat_lat, double sat_lon, double sat_alt
    );
    
    static double calculateRadialVelocity(
        double ant_lat, double ant_lon, double ant_alt,
        double sat_lat, double sat_lon, double sat_alt,
        double sat_velocity, double heading
    );
};

class doppler_simulator_impl : public doppler_simulator
{
private:
    // Constants
    static constexpr double EARTH_RADIUS = 6371.0;    // km
    static constexpr double NOAA_ALTITUDE = 850.0;    // km
    static constexpr double LIGHT_SPEED = 299792.458; // km/s
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEG_TO_RAD = PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / PI;
    static constexpr double ORBITAL_PERIOD = 101.5;   // minutes (NOAA typical)
    static constexpr double MIN_ELEVATION = -10.0;
    static constexpr double MAX_ELEVATION = 90.0;

    // Configuration parameters
    float d_sample_rate;
    float d_center_freq;
    float d_range_km;
    float d_sat_velocity;
    bool d_auto_trim;
    float d_trim_threshold;
    
    // Ground station parameters
    float d_antenna_lat;
    float d_antenna_lon;
    float d_antenna_alt;
    AntennaPattern d_antenna_pattern;
    
    // Satellite parameters
    float d_orbital_inclination;
    bool d_ascending;
    float d_start_lat;
    float d_start_lon;
    float d_current_lat;
    float d_current_lon;
    double d_elapsed_time;
    bool d_loop_enabled;
    
    // Current status
    float d_current_doppler;
    float d_current_strength;
    float d_current_elevation;
    float d_current_azimuth;
    float d_current_distance;
    float d_current_velocity;
    float d_current_phase;
    float d_last_doppler_phase;

    void update_position(double dt);
    void calculate_look_angles();
    void calculate_signal_parameters();
    double calculate_orbital_heading() const;
    void update_status();
    
    void calculate_satellite_position(double& x, double& y, double& z) const;
    void calculate_antenna_position(double& x, double& y, double& z) const;
    void calculate_velocity_vector(double& vx, double& vy, double& vz) const;

public:
    doppler_simulator_impl(float sample_rate,
                         float center_freq,
                         float range_km,
                         float sat_velocity,
                         bool auto_trim,
                         float trim_threshold,
                         float antenna_lat,
                         float antenna_lon,
                         float antenna_alt,
                         float start_lat,
                         float start_lon,
                         float orbital_inclination,
                         bool ascending);
    ~doppler_simulator_impl() = default;

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);
    int work(int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
            
    // Parameter setters
    void set_range(float range_km) override { d_range_km = range_km; }
    void set_velocity(float velocity) override { d_sat_velocity = velocity; }
    void set_antenna_position(float lat, float lon, float alt) override;
    void set_auto_trim(bool enable) override { d_auto_trim = enable; }
    void set_antenna_type(AntennaType type) override { d_antenna_pattern.set_type(type); }
    void set_antenna_orientation(float azimuth, float elevation) override {
        d_antenna_pattern.set_orientation(azimuth, elevation);
    }
    
    // Status getters
    float get_doppler_shift() const override { return d_current_doppler; }
    float get_signal_strength() const override { return d_current_strength; }
    float get_elevation() const override { return d_current_elevation; }
    float get_azimuth() const override { return d_current_azimuth; }
    float get_current_distance() const { return d_current_distance; }
    float get_current_velocity() const { return d_current_velocity; }
    AntennaType get_antenna_type() const override { return d_antenna_pattern.get_type(); }
    float get_current_lat() const override { return d_current_lat; }
    float get_current_lon() const override { return d_current_lon; }
    float get_current_heading() const override;
    float get_orbital_period() const override { return ORBITAL_PERIOD; }
};

} // namespace apt_encoder
} // namespace gr

#endif