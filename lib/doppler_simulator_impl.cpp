#include "doppler_simulator_impl.hpp"
#include <gnuradio/io_signature.h>
#include <cmath>

namespace gr {
namespace apt_encoder {

doppler_simulator::sptr doppler_simulator::make(float sample_rate,
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
                                             bool ascending)
{
    return gnuradio::make_block_sptr<doppler_simulator_impl>(
        sample_rate, center_freq, range_km, sat_velocity, auto_trim, 
        trim_threshold, antenna_lat, antenna_lon, antenna_alt,
        start_lat, start_lon, orbital_inclination, ascending);
}

doppler_simulator_impl::doppler_simulator_impl(float sample_rate,
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
                                            bool ascending)
    : gr::sync_block("doppler_simulator",
                     gr::io_signature::make(1, 1, sizeof(float)),
                     gr::io_signature::make(1, 1, sizeof(float))),
      d_sample_rate(sample_rate),
      d_center_freq(center_freq),
      d_range_km(range_km),
      d_sat_velocity(sat_velocity),
      d_auto_trim(auto_trim),
      d_trim_threshold(trim_threshold),
      d_antenna_lat(antenna_lat),
      d_antenna_lon(antenna_lon),
      d_antenna_alt(antenna_alt),
      d_orbital_inclination(orbital_inclination),
      d_ascending(ascending),
      d_start_lat(start_lat),
      d_start_lon(start_lon),
      d_current_lat(start_lat),
      d_current_lon(start_lon),
      d_elapsed_time(0.0),
      d_loop_enabled(true),
      d_current_doppler(1.0),
      d_current_strength(1.0),
      d_current_elevation(0.0),
      d_current_azimuth(0.0)
{
    message_port_register_out(pmt::intern("sat_pos"));
    calculate_look_angles();
    update_status();
}

void doppler_simulator_impl::set_antenna_position(float lat, float lon, float alt)
{
    d_antenna_lat = lat;
    d_antenna_lon = lon;
    d_antenna_alt = alt;
    calculate_look_angles();
    update_status();
}

void doppler_simulator_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
}

void doppler_simulator_impl::update_position(double dt)
{
    double orbit_angle = d_orbital_inclination - 90.0;
    
    double heading_rad = calculate_orbital_heading() * DEG_TO_RAD;
    
    double v_north = d_sat_velocity * std::cos(heading_rad);
    double v_east = d_sat_velocity * std::sin(heading_rad);
    
    double delta_lat = (v_north * dt) / EARTH_RADIUS;
    double delta_lon = (v_east * dt) / (EARTH_RADIUS * std::cos(d_current_lat * DEG_TO_RAD));
    
    d_current_lat += delta_lat * RAD_TO_DEG;
    d_current_lon += delta_lon * RAD_TO_DEG;
    
    if (d_ascending && d_current_lat > 82.0) {
        d_ascending = false;
        d_current_lon += orbit_angle * 2.0;
    }
    else if (!d_ascending && d_current_lat < -82.0) {
        d_ascending = true;
        d_current_lon += orbit_angle * 2.0;
    }
    
    while (d_current_lon > 180.0) d_current_lon -= 360.0;
    while (d_current_lon < -180.0) d_current_lon += 360.0;
}

double doppler_simulator_impl::calculate_orbital_heading() const
{
    double orbit_angle = d_orbital_inclination - 90.0;
    
    double heading;
    if (d_ascending) {
        heading = orbit_angle;
    } else {
        heading = 180.0 - orbit_angle;
    }
    
    while (heading < 0.0) heading += 360.0;
    while (heading >= 360.0) heading -= 360.0;
    
    return heading;
}

void doppler_simulator_impl::calculate_look_angles()
{
    // Convert to radians
    double ant_lat = d_antenna_lat * DEG_TO_RAD;
    double ant_lon = d_antenna_lon * DEG_TO_RAD;
    double sat_lat = d_current_lat * DEG_TO_RAD;
    double sat_lon = d_current_lon * DEG_TO_RAD;
    
    // Calculate positions with altitude
    double ant_radius = EARTH_RADIUS + (d_antenna_alt / 1000.0); // convert m to km
    double sat_radius = EARTH_RADIUS + NOAA_ALTITUDE;
    
    // Calculate satellite position relative to Earth center
    double sx = sat_radius * std::cos(sat_lat) * std::cos(sat_lon);
    double sy = sat_radius * std::cos(sat_lat) * std::sin(sat_lon);
    double sz = sat_radius * std::sin(sat_lat);
    
    // Calculate antenna position
    double ax = ant_radius * std::cos(ant_lat) * std::cos(ant_lon);
    double ay = ant_radius * std::cos(ant_lat) * std::sin(ant_lon);
    double az = ant_radius * std::sin(ant_lat);
    
    // Calculate relative vector (satellite to antenna)
    double rx = sx - ax;
    double ry = sy - ay;
    double rz = sz - az;
    double range = std::sqrt(rx*rx + ry*ry + rz*rz);
    
    // Calculate local coordinate system at antenna position
    // North vector
    double north_x = -std::sin(ant_lat) * std::cos(ant_lon);
    double north_y = -std::sin(ant_lat) * std::sin(ant_lon);
    double north_z = std::cos(ant_lat);
    
    // East vector
    double east_x = -std::sin(ant_lon);
    double east_y = std::cos(ant_lon);
    double east_z = 0;
    
    // Up vector (radial from Earth center through antenna)
    double up_x = std::cos(ant_lat) * std::cos(ant_lon);
    double up_y = std::cos(ant_lat) * std::sin(ant_lon);
    double up_z = std::sin(ant_lat);
    
    // Calculate antenna pointing vector based on azimuth and elevation
    double ant_az_rad = d_antenna_pattern.get_azimuth_offset() * DEG_TO_RAD;
    double ant_el_rad = d_antenna_pattern.get_elevation_offset() * DEG_TO_RAD;
    
    // Convert antenna pointing from az/el to xyz
    // First, start with vector pointing north
    double ant_vec_x = north_x;
    double ant_vec_y = north_y;
    double ant_vec_z = north_z;
    
    // Rotate around up vector by azimuth
    double cos_az = std::cos(ant_az_rad);
    double sin_az = std::sin(ant_az_rad);
    double temp_x = ant_vec_x * cos_az - east_x * sin_az;
    double temp_y = ant_vec_y * cos_az - east_y * sin_az;
    double temp_z = ant_vec_z * cos_az - east_z * sin_az;
    ant_vec_x = temp_x;
    ant_vec_y = temp_y;
    ant_vec_z = temp_z;
    
    // Rotate around horizontal axis by elevation
    double cos_el = std::cos(ant_el_rad);
    double sin_el = std::sin(ant_el_rad);
    ant_vec_x = ant_vec_x * cos_el + up_x * sin_el;
    ant_vec_y = ant_vec_y * cos_el + up_y * sin_el;
    ant_vec_z = ant_vec_z * cos_el + up_z * sin_el;
    
    // Calculate satellite azimuth and elevation relative to antenna position
    // Project relative vector onto local horizontal plane
    double proj_north = rx*north_x + ry*north_y + rz*north_z;
    double proj_east = rx*east_x + ry*east_y + rz*east_z;
    double proj_up = rx*up_x + ry*up_y + rz*up_z;
    
    // Calculate true elevation and azimuth
    d_current_elevation = std::asin(proj_up / range) * RAD_TO_DEG;
    d_current_azimuth = std::atan2(proj_east, proj_north) * RAD_TO_DEG;
    if (d_current_azimuth < 0) d_current_azimuth += 360.0;
    
    // Calculate angle between antenna pointing vector and satellite direction
    double rx_norm = rx / range;
    double ry_norm = ry / range;
    double rz_norm = rz / range;
    
    double angle_to_satellite = std::acos(
        ant_vec_x * rx_norm +
        ant_vec_y * ry_norm +
        ant_vec_z * rz_norm
    ) * RAD_TO_DEG;
    
    float angle_factor = std::cos(angle_to_satellite * DEG_TO_RAD);
    angle_factor = std::max(0.0f, angle_factor);  // 負の値は0にクリップ
    
    float distance_factor = static_cast<float>(1.0 / (range * range));
    float max_range = EARTH_RADIUS + NOAA_ALTITUDE;
    float normalized_distance = distance_factor * (max_range * max_range);
    
    float total_strength = angle_factor * normalized_distance;
    
    if (d_auto_trim && d_current_elevation < d_trim_threshold) {
        total_strength = 0.0f;
    }
    
    total_strength = std::max(0.0f, std::min(1.0f, total_strength));
    
    d_current_strength = total_strength;
}


void doppler_simulator_impl::update_status()
{
    calculate_look_angles();
    
    double ant_lat_rad = d_antenna_lat * DEG_TO_RAD;
    double ant_lon_rad = d_antenna_lon * DEG_TO_RAD;
    double sat_lat_rad = d_current_lat * DEG_TO_RAD;
    double sat_lon_rad = d_current_lon * DEG_TO_RAD;
    
    double ant_radius = EARTH_RADIUS + (d_antenna_alt / 1000.0);
    double sat_radius = EARTH_RADIUS + NOAA_ALTITUDE;
    
    double ant_x = ant_radius * std::cos(ant_lat_rad) * std::cos(ant_lon_rad);
    double ant_y = ant_radius * std::cos(ant_lat_rad) * std::sin(ant_lon_rad);
    double ant_z = ant_radius * std::sin(ant_lat_rad);
    
    double sat_x = sat_radius * std::cos(sat_lat_rad) * std::cos(sat_lon_rad);
    double sat_y = sat_radius * std::cos(sat_lat_rad) * std::sin(sat_lon_rad);
    double sat_z = sat_radius * std::sin(sat_lat_rad);
    
    double rx = sat_x - ant_x;
    double ry = sat_y - ant_y;
    double rz = sat_z - ant_z;
    double range = std::sqrt(rx*rx + ry*ry + rz*rz);
    
    rx /= range;
    ry /= range;
    rz /= range;
    
    double heading_rad = calculate_orbital_heading() * DEG_TO_RAD;

    double north_x = -std::sin(sat_lat_rad) * std::cos(sat_lon_rad);
    double north_y = -std::sin(sat_lat_rad) * std::sin(sat_lon_rad);
    double north_z = std::cos(sat_lat_rad);
    
    double east_x = -std::sin(sat_lon_rad);
    double east_y = std::cos(sat_lon_rad);
    double east_z = 0.0;
    
    // doppler shift calc
    double vx = d_sat_velocity * (north_x * std::cos(heading_rad) + east_x * std::sin(heading_rad));
    double vy = d_sat_velocity * (north_y * std::cos(heading_rad) + east_y * std::sin(heading_rad));
    double vz = d_sat_velocity * (north_z * std::cos(heading_rad) + east_z * std::sin(heading_rad));
    
    double vr = rx * vx + ry * vy + rz * vz;
    d_current_doppler = static_cast<float>(1.0 - vr / LIGHT_SPEED);
    d_current_velocity = static_cast<float>(vr);
    d_current_distance = static_cast<float>(range);
    
    float ant_gain = d_antenna_pattern.calculate_gain(d_current_azimuth, d_current_elevation);
    
    // Attenaution
    double distance_factor = 1.0 / (range * range);
    double max_range = EARTH_RADIUS + NOAA_ALTITUDE;
    double normalized_distance = distance_factor * (max_range * max_range);
    
    // Final signal strength 
    float strength = ant_gain * static_cast<float>(normalized_distance);
    
    // Auto Trim (maybe did not work)
    if (d_auto_trim && d_current_elevation < d_trim_threshold) {
        strength = 0.0f;
    }
    
    // Normallize
    d_current_strength = std::max(0.0f, std::min(1.0f, strength));
    
    // Messages
    pmt::pmt_t pos_dict = pmt::make_dict();
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("latitude"), 
        pmt::from_double(d_current_lat));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("longitude"), 
        pmt::from_double(d_current_lon));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("elevation"), 
        pmt::from_double(d_current_elevation));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("azimuth"), 
        pmt::from_double(d_current_azimuth));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("doppler"), 
        pmt::from_double(d_current_doppler));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("strength"), 
        pmt::from_double(d_current_strength));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("range"), 
        pmt::from_double(d_current_distance));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("velocity"), 
        pmt::from_double(d_current_velocity));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("heading"), 
        pmt::from_double(calculate_orbital_heading()));
    
    // Add antenna orientation information
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("antennaAzimuth"), 
        pmt::from_double(d_antenna_pattern.get_azimuth_offset()));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("antennaElevation"), 
        pmt::from_double(d_antenna_pattern.get_elevation_offset()));
    
    // Add ground station information
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("antennaLat"), 
        pmt::from_double(d_antenna_lat));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("antennaLon"), 
        pmt::from_double(d_antenna_lon));
    pos_dict = pmt::dict_add(pos_dict, pmt::intern("antennaAlt"), 
        pmt::from_double(d_antenna_alt));
    
    message_port_pub(pmt::intern("sat_pos"), pos_dict);
}

float doppler_simulator_impl::get_current_heading() const
{
    return calculate_orbital_heading();
}

int doppler_simulator_impl::work(int noutput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
{
    const float *in = (const float *) input_items[0];
    float *out = (float *) output_items[0];

    // Update position and status every 100ms
    double dt = noutput_items / static_cast<double>(d_sample_rate);
    d_elapsed_time += dt;
    
    if (d_loop_enabled) {
        update_position(dt);
        
        // Update status every 100ms
        static double last_status_time = 0.0;
        if (d_elapsed_time - last_status_time >= 0.1) {
            update_status();
            last_status_time = d_elapsed_time;
        }
    }

    // Apply Doppler effect and signal strength
    for (int i = 0; i < noutput_items; i++) {
        // Calculate phase shift for Doppler effect
        float phase_shift = (d_current_doppler - 1.0) * d_center_freq * 2.0 * PI * 
                           (d_elapsed_time + i/static_cast<double>(d_sample_rate));
                           
        // Apply Doppler shift and signal strength
        float shifted_signal = in[i] * std::cos(phase_shift);
        out[i] = shifted_signal * d_current_strength;
        
        // Ensure output is within bounds
        out[i] = std::max(-1.0f, std::min(1.0f, out[i]));
    }
    
    return noutput_items;
}

} // namespace apt_encoder
} // namespace gr