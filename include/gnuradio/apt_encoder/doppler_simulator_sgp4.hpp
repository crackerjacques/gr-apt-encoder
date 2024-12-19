#ifndef INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_SGP4_HPP
#define INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_SGP4_HPP

#include <gnuradio/sync_block.h>
#include <gnuradio/apt_encoder/api.hpp>
#include <gnuradio/apt_encoder/doppler_simulator.hpp>
#include <string>
#include <ctime>

namespace gr {
namespace apt_encoder {

/*!
 * \brief SGP4 Doppler effect simulator block
 *
 * This block simulates Doppler shift and signal attenuation based on accurate
 * satellite orbit calculations using the SGP4 orbit propagator with TLE data.
 */
class GR_APT_ENCODER_API doppler_simulator_sgp4 : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<doppler_simulator_sgp4> sptr;

    /*!
     * \brief Create an SGP4 Doppler simulator
     *
     * \param sample_rate Input signal sample rate
     * \param center_freq Carrier frequency
     * \param tle_line1 First line of TLE
     * \param tle_line2 Second line of TLE
     * \param antenna_lat Ground station latitude (-90 to 90 degrees)
     * \param antenna_lon Ground station longitude (-180 to 180 degrees)
     * \param antenna_alt Ground station altitude in meters
     */
    static sptr make(float sample_rate,
                    float center_freq,
                    const std::string& tle_line1,
                    const std::string& tle_line2,
                    float antenna_lat = 35.0f,
                    float antenna_lon = 135.0f,
                    float antenna_alt = 0.0f);

    // Ground station setup
    virtual void set_antenna_position(float lat, float lon, float alt) = 0;
    virtual void set_tle(const std::string& line1, const std::string& line2) = 0;
    virtual void set_antenna_type(AntennaType type) = 0;
    virtual void set_antenna_orientation(float azimuth, float elevation) = 0;

    // Status getters
    virtual float get_doppler_shift() const = 0;
    virtual float get_signal_strength() const = 0;
    virtual float get_elevation() const = 0;
    virtual float get_azimuth() const = 0;
    virtual float get_range() const = 0;
    virtual float get_velocity() const = 0;
    virtual float get_heading() const = 0;

    // Simulation time control
    /*!
     * \brief Set simulation time
     *
     * \param year Full year (e.g., 2024)
     * \param month Month (1-12)
     * \param day Day of month (1-31)
     * \param hour Hour (0-23)
     * \param minute Minute (0-59)
     * \param second Second (0-59.999)
     */
    virtual void set_simulation_time(int year, int month, int day,
                                   int hour, int minute, double second) = 0;

    /*!
     * \brief Set simulation start time using Unix timestamp
     *
     * \param unix_time Unix timestamp (seconds since epoch)
     */
    virtual void set_simulation_start_time(time_t unix_time) = 0;

    /*!
     * \brief Enable/disable realtime mode
     *
     * \param enable True for realtime, false for simulated time
     */
    virtual void set_realtime_mode(bool enable) = 0;

    /*!
     * \brief Get current simulation time
     *
     * \return Current Unix timestamp in simulation
     */
    virtual time_t get_simulation_time() const = 0;

protected:
    doppler_simulator_sgp4() {}  // Protected constructor
};

} // namespace apt_encoder
} // namespace gr

#endif /* INCLUDED_APT_ENCODER_DOPPLER_SIMULATOR_SGP4_HPP */