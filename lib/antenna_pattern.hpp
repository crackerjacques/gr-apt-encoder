#ifndef INCLUDED_APT_ENCODER_ANTENNA_PATTERN_HPP
#define INCLUDED_APT_ENCODER_ANTENNA_PATTERN_HPP

#include <cmath>
#include <string>
#include <memory>
#include <gnuradio/apt_encoder/doppler_simulator.hpp>  // Using AntennaType definition

namespace gr {
namespace apt_encoder {

/*!
 * \brief Antenna pattern simulation class
 *
 * Simulates various antenna radiation patterns for satellite signal reception.
 */
class AntennaPattern {
public:
    static constexpr double PI = 3.14159265358979323846;

    /*!
     * \brief Constructor
     * \param type Initial antenna type
     */
    AntennaPattern(AntennaType type = AntennaType::OMNI);
    virtual ~AntennaPattern() = default;

    /*!
     * \brief Calculate gain from azimuth and elevation angles
     * \param azimuth Azimuth angle (degrees)
     * \param elevation Elevation angle (degrees)
     * \return Antenna gain (0.0-1.0)
     */
    double calculate_gain(double azimuth, double elevation) const;
    
    /*!
     * \brief Set antenna type
     * \param type Type of antenna to simulate
     */
    void set_type(AntennaType type);

    /*!
     * \brief Get current antenna type
     * \return Current antenna type
     */
    AntennaType get_type() const { return type_; }
    
    /*!
     * \brief Set antenna orientation
     * \param azimuth Azimuth angle (degrees)
     * \param elevation Elevation angle (degrees)
     */
    void set_orientation(double azimuth, double elevation);

    /*!
     * \brief Get current azimuth offset
     * \return Current azimuth angle in degrees
     */
    double get_azimuth_offset() const { return azimuth_offset_; }

    /*!
     * \brief Get current elevation offset
     * \return Current elevation angle in degrees
     */
    double get_elevation_offset() const { return elevation_offset_; }

private:
    AntennaType type_;               // Current antenna type
    double azimuth_offset_;          // Antenna azimuth offset (degrees)
    double elevation_offset_;        // Antenna elevation offset (degrees)

    // Gain pattern calculations for each antenna type
    double calculate_dipole_gain(double azimuth, double elevation) const;
    double calculate_double_cross_gain(double azimuth, double elevation) const;
    double calculate_qfh_gain(double azimuth, double elevation) const;
    double calculate_turnstile_gain(double azimuth, double elevation) const;

    // Helper functions
    double deg_to_rad(double degrees) const { return degrees * PI / 180.0; }
    double normalize_angle(double angle) const;
};

} // namespace apt_encoder
} // namespace gr

#endif // INCLUDED_APT_ENCODER_ANTENNA_PATTERN_HPP