#include "antenna_pattern.hpp"
#include <algorithm>

namespace gr {
namespace apt_encoder {

AntennaPattern::AntennaPattern(AntennaType type)
    : type_(type)
    , azimuth_offset_(0.0)
    , elevation_offset_(0.0)
{
}

void AntennaPattern::set_type(AntennaType type)
{
    type_ = type;
}


void AntennaPattern::set_orientation(double azimuth, double elevation)
{
    azimuth_offset_ = normalize_angle(azimuth);
    elevation_offset_ = std::clamp(elevation, -90.0, 90.0);
}

double AntennaPattern::normalize_angle(double angle) const
{
    angle = std::fmod(angle, 360.0);
    if (angle < 0) {
        angle += 360.0;
    }
    return angle;
}

double AntennaPattern::calculate_gain(double azimuth, double elevation) const
{
    double rel_azimuth = normalize_angle(azimuth - azimuth_offset_);
    double rel_elevation = elevation - elevation_offset_;

    switch (type_) {
        case AntennaType::DIPOLE:
            return calculate_dipole_gain(rel_azimuth, rel_elevation);
        case AntennaType::DOUBLE_CROSS:
            return calculate_double_cross_gain(rel_azimuth, rel_elevation);
        case AntennaType::QFH:
            return calculate_qfh_gain(rel_azimuth, rel_elevation);
        case AntennaType::TURNSTILE:
            return calculate_turnstile_gain(rel_azimuth, rel_elevation);
        case AntennaType::OMNI:
        default:
            return 1.0;
    }
}

double AntennaPattern::calculate_dipole_gain(double azimuth, double elevation) const
{
    double theta = deg_to_rad(90.0 - elevation);
    double phi = deg_to_rad(azimuth);
    double gain = std::cos(theta) * std::cos(theta);

    if (elevation < 0) {
        gain *= 0.3;
    }
    
    return std::max(0.0, gain);
}

double AntennaPattern::calculate_double_cross_gain(double azimuth, double elevation) const
{
    double theta = deg_to_rad(90.0 - elevation);
    double phi = deg_to_rad(azimuth);
    double gain1 = std::pow(std::cos(theta), 2) * std::pow(std::cos(phi), 2);
    double gain2 = std::pow(std::cos(theta), 2) * std::pow(std::sin(phi), 2);
    double total_gain = std::sqrt(gain1 + gain2);
    double elevation_factor = std::sin(deg_to_rad(elevation));
    total_gain *= (elevation_factor + 1.0) / 2.0;
    
    return std::max(0.0, total_gain);
}

double AntennaPattern::calculate_qfh_gain(double azimuth, double elevation) const
{
    double theta = deg_to_rad(90.0 - elevation);
    double elevation_gain = std::cos(theta / 2.0); // 天頂で最大

    if (elevation < 30.0) {
        elevation_gain *= 0.8 + 0.2 * (elevation / 30.0);
    }
    
    if (elevation < 0) {
        elevation_gain *= std::exp(elevation / 10.0);
    }
    
    return std::max(0.0, elevation_gain);
}

double AntennaPattern::calculate_turnstile_gain(double azimuth, double elevation) const
{
    double theta = deg_to_rad(90.0 - elevation);
    double phi = deg_to_rad(azimuth);
    
    double gain = std::pow(std::cos(theta), 2);
    
    gain *= (1.0 + 0.2 * std::cos(2.0 * phi));
    
    if (elevation < 0) {
        gain *= std::exp(elevation / 15.0);
    }
    
    return std::max(0.0, gain);
}

} // namespace apt_encoder
} // namespace gr