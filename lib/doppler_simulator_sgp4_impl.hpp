#ifndef INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_SGP4_IMPL_HPP
#define INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_SGP4_IMPL_HPP

#include <gnuradio/apt_encoder/doppler_simulator_sgp4.hpp>
#include "sgp4_orbit.hpp"
#include "antenna_pattern.hpp"

namespace gr {
namespace apt_encoder {

class doppler_simulator_sgp4_impl : public doppler_simulator_sgp4
{
private:
    // Parameters
    float sample_rate_;
    float center_freq_;
    float current_time_;

    // Current state
    float current_doppler_;
    float current_strength_;
    float current_elevation_;
    float current_azimuth_;
    float current_range_;
    float current_velocity_;
    float current_heading_;

    // SGP4 orbit calculator
    std::unique_ptr<SGP4Orbit> orbit_;
    AntennaPattern antenna_;

    // time
    bool realtime_mode_;
    time_t simulation_start_time_;
    time_t simulation_offset_;

    // Update position and status
    void update_position(double unix_time);
    void publish_status();

public:
    doppler_simulator_sgp4_impl(float sample_rate,
                              float center_freq,
                              const std::string& tle_line1,
                              const std::string& tle_line2,
                              float antenna_lat,
                              float antenna_lon,
                              float antenna_alt);

    ~doppler_simulator_sgp4_impl() override;

    // GR::sync_block implementation
    void forecast(int noutput_items,
                 gr_vector_int &ninput_items_required) override;
    int work(int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items) override;

    // Parameter setters
    void set_antenna_position(float lat, float lon, float alt) override;
    void set_tle(const std::string& line1, const std::string& line2) override;
    void set_antenna_type(AntennaType type) override;
    void set_antenna_orientation(float azimuth, float elevation) override;

    // time
    void set_simulation_time(int year, int month, int day,
                           int hour, int minute, double second) override;
    void set_simulation_start_time(time_t unix_time) override;
    void set_realtime_mode(bool enable) override;
    time_t get_simulation_time() const override;

    // Status getters
    float get_doppler_shift() const override { return current_doppler_; }
    float get_signal_strength() const override { return current_strength_; }
    float get_elevation() const override { return current_elevation_; }
    float get_azimuth() const override { return current_azimuth_; }
    float get_range() const override { return current_range_; }
    float get_velocity() const override { return current_velocity_; }
    float get_heading() const override { return current_heading_; }
};

} // namespace apt_encoder
} // namespace gr

#endif /* INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_SGP4_IMPL_HPP */