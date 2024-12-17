// include/gnuradio/apt_encoder/doppler_simulator.hpp

#ifndef INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_H
#define INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_H

#include <gnuradio/apt_encoder/api.hpp>
#include <gnuradio/sync_block.h>

namespace gr {
namespace apt_encoder {

enum class AntennaType {
    OMNI,
    DIPOLE,
    DOUBLE_CROSS,
    QFH,
    TURNSTILE
    };

class GR_APT_ENCODER_API doppler_simulator : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<doppler_simulator> sptr;

    static sptr make(float sample_rate,
                    float center_freq,
                    float range_km,
                    float sat_velocity = 7.8,
                    bool auto_trim = true,
                    float trim_threshold = 20.0,
                    float antenna_lat = 35.0,
                    float antenna_lon = 135.0,
                    float antenna_alt = 0.0,
                    float start_lat = 0.0,
                    float start_lon = 0.0,
                    float orbital_inclination = 98.7,
                    bool ascending = true);

    virtual void set_range(float range_km) = 0;
    virtual void set_velocity(float velocity) = 0;
    virtual void set_antenna_position(float lat, float lon, float alt) = 0;
    virtual void set_auto_trim(bool enable) = 0;
    virtual void set_antenna_type(AntennaType type) = 0;
    virtual void set_antenna_orientation(float azimuth, float elevation) = 0;

    // get common parameter
    virtual float get_doppler_shift() const = 0;
    virtual float get_signal_strength() const = 0;
    virtual float get_elevation() const = 0;
    virtual float get_azimuth() const = 0;
    virtual AntennaType get_antenna_type() const = 0;
    virtual float get_current_lat() const = 0;
    virtual float get_current_lon() const = 0;
    virtual float get_current_heading() const = 0;
    virtual float get_orbital_period() const = 0;

    virtual float get_current_distance() const = 0;
    virtual float get_current_velocity() const = 0;
};

} // namespace apt_encoder
} // namespace gr

#endif