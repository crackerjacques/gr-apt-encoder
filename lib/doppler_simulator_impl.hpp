#ifndef INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_IMPL_H
#define INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_IMPL_H

#include <gnuradio/apt_encoder/doppler_simulator.hpp>

namespace gr {
namespace apt_encoder {

class doppler_simulator_impl : public doppler_simulator
{
private:
    float d_sample_rate;
    float d_center_freq;
    float d_range_km;
    float d_sat_velocity;
    bool d_auto_trim;
    float d_trim_threshold;
    
    // Message Input
    double d_duration;
    uint64_t d_total_samples;
    double d_path_length;
    bool d_params_set;

    // status
    uint64_t d_processed_samples;
    float d_current_doppler;
    float d_current_strength;

    // constants
    static constexpr double EARTH_RADIUS = 6371.0;  // km
    static constexpr double LIGHT_SPEED = 299792.458;  // km/s
    static constexpr double PI = 3.14159265358979323846;

    // helper
    double calculate_elevation(double t) const;
    double calculate_doppler_shift(double t) const;
    double calculate_signal_strength(double elevation) const;
    void handle_duration_msg(pmt::pmt_t msg);
    void handle_samples_msg(pmt::pmt_t msg);
    void handle_path_length_msg(pmt::pmt_t msg);
    void update_status(double t);

public:
    doppler_simulator_impl(float sample_rate,
                         float center_freq,
                         float range_km,
                         float sat_velocity,
                         bool auto_trim,
                         float trim_threshold);
    ~doppler_simulator_impl() = default;

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);
    
    int work(int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
            
    void set_range(float range_km) override;
    void set_velocity(float velocity) override;
    float get_doppler_shift() const override { return d_current_doppler; }
    float get_signal_strength() const override { return d_current_strength; }
    void set_auto_trim(bool enable) override;
};

} // namespace apt_encoder
} // namespace gr

#endif /* INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_IMPL_H */