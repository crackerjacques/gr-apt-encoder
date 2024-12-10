#ifndef INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_H
#define INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_H

#include <gnuradio/apt_encoder/api.hpp>
#include <gnuradio/sync_block.h>

namespace gr {
namespace apt_encoder {

class GR_APT_ENCODER_API doppler_simulator : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<doppler_simulator> sptr;

    static sptr make(float sample_rate,
                    float center_freq,
                    float range_km,
                    float sat_velocity = 7.8,
                    bool auto_trim = true,
                    float trim_threshold = 20.0);

    virtual void set_range(float range_km) = 0;
    virtual void set_velocity(float velocity) = 0;
    virtual float get_doppler_shift() const = 0;
    virtual float get_signal_strength() const = 0;
    virtual void set_auto_trim(bool enable) = 0;
};

} // namespace apt_encoder
} // namespace gr

#endif