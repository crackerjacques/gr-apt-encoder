#include "doppler_simulator_sgp4_impl.hpp"
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <cmath>
#include <ctime>
#include <iomanip>

namespace gr {
namespace apt_encoder {


doppler_simulator_sgp4::sptr doppler_simulator_sgp4::make(
    float sample_rate,
    float center_freq,
    const std::string& tle_line1,
    const std::string& tle_line2,
    float antenna_lat,
    float antenna_lon,
    float antenna_alt)
{
    return gnuradio::make_block_sptr<doppler_simulator_sgp4_impl>(
        sample_rate, center_freq, tle_line1, tle_line2,
        antenna_lat, antenna_lon, antenna_alt);
}

doppler_simulator_sgp4_impl::~doppler_simulator_sgp4_impl()
{

}

doppler_simulator_sgp4_impl::doppler_simulator_sgp4_impl(
    float sample_rate,
    float center_freq,
    const std::string& tle_line1,
    const std::string& tle_line2,
    float antenna_lat,
    float antenna_lon,
    float antenna_alt)
    : gr::sync_block("doppler_simulator_sgp4",
                     gr::io_signature::make(0, 1, sizeof(float)),  // 0-1の入力を許可
                     gr::io_signature::make(1, 1, sizeof(float))),
      sample_rate_(sample_rate),
      center_freq_(center_freq),
      current_time_(0),
      current_doppler_(1.0),
      current_strength_(1.0),
      current_elevation_(0.0),
      current_azimuth_(0.0),
      current_range_(0.0),
      current_velocity_(0.0),
      current_heading_(0.0),
      realtime_mode_(true),
      simulation_start_time_(std::time(nullptr)),
      simulation_offset_(0)
{
    try {
        // Initialize message port
        message_port_register_out(pmt::intern("sat_pos"));

        // Initialize orbit calculator
        orbit_ = std::make_unique<SGP4Orbit>();
        orbit_->set_tle(tle_line1, tle_line2);

        // Initialize ground station
        SGP4Orbit::GroundStation station;
        station.latitude = antenna_lat;
        station.longitude = antenna_lon;
        station.altitude = antenna_alt;
        orbit_->set_ground_station(station);

        // Initialize antenna
        antenna_.set_type(AntennaType::OMNI);
        antenna_.set_orientation(90.0, 10.0);

        // Calculate initial position
        update_position(get_simulation_time());

        std::cerr << "Doppler Simulator SGP4 initialized:\n"
                  << "  Sample rate: " << sample_rate_ << " Hz\n"
                  << "  Center frequency: " << center_freq_ << " Hz\n"
                  << "  Ground station: " << antenna_lat << "°N, "
                  << antenna_lon << "°E, " << antenna_alt << "m\n"
                  << "  TLE Line 1: " << tle_line1 << "\n"
                  << "  TLE Line 2: " << tle_line2 << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error initializing Doppler simulator: " << e.what() << std::endl;
        throw;
    }
}

void doppler_simulator_sgp4_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
}

int doppler_simulator_sgp4_impl::work(
    int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    float *out = static_cast<float*>(output_items[0]);
    const float *in = input_items.size() > 0 ? static_cast<const float*>(input_items[0]) : nullptr;

    try {
        double dt = noutput_items / static_cast<double>(sample_rate_);
        current_time_ += dt;

        // Update position every 10ms
        static double last_update_time = 0.0;
        if (current_time_ - last_update_time >= 0.01) {
            time_t current_sim_time = get_simulation_time();
            update_position(current_sim_time);
            publish_status();
            last_update_time = current_time_;
        }

        // Process signal
        for (int i = 0; i < noutput_items; i++) {
            float phase_shift = (current_doppler_ - 1.0f) * center_freq_ * 2.0f * M_PI * 
                              (current_time_ + i/static_cast<float>(sample_rate_));
            
            float signal;
            if (in) {
                signal = in[i] * std::cos(phase_shift);
            } else {
                signal = std::sin(2.0f * M_PI * center_freq_ * 
                               (current_time_ + i/static_cast<float>(sample_rate_)));
            }
            
            out[i] = signal * current_strength_;
            out[i] = std::max(-1.0f, std::min(1.0f, out[i]));
        }

        return noutput_items;
    }
    catch (const std::exception& e) {
        std::cerr << "Error in work function: " << e.what() << std::endl;
        return 0;
    }
}

void doppler_simulator_sgp4_impl::set_antenna_position(float lat, float lon, float alt)
{
    try {
        SGP4Orbit::GroundStation station;
        station.latitude = lat;
        station.longitude = lon;
        station.altitude = alt;
        orbit_->set_ground_station(station);
        update_position(get_simulation_time());

        std::cerr << "Updated ground station position: "
                  << lat << "°N, " << lon << "°E, " << alt << "m" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error setting antenna position: " << e.what() << std::endl;
        throw;
    }
}

void doppler_simulator_sgp4_impl::set_tle(const std::string& line1, const std::string& line2)
{
    try {
        orbit_->set_tle(line1, line2);
        update_position(get_simulation_time());
        std::cerr << "Updated TLE data:\n  " << line1 << "\n  " << line2 << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error setting TLE: " << e.what() << std::endl;
        throw;
    }
}

void doppler_simulator_sgp4_impl::set_antenna_type(AntennaType type)
{
    try {
        antenna_.set_type(type);
        update_position(get_simulation_time());
        std::cerr << "Updated antenna type" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error setting antenna type: " << e.what() << std::endl;
        throw;
    }
}

void doppler_simulator_sgp4_impl::set_antenna_orientation(float azimuth, float elevation)
{
    try {
        antenna_.set_orientation(azimuth, elevation);
        update_position(get_simulation_time());
        std::cerr << "Updated antenna orientation: Az=" << azimuth 
                  << "°, El=" << elevation << "°" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error setting antenna orientation: " << e.what() << std::endl;
        throw;
    }
}

void doppler_simulator_sgp4_impl::set_simulation_time(
    int year, int month, int day,
    int hour, int minute, double second)
{
    try {
        std::tm timeinfo = {};
        timeinfo.tm_year = year - 1900;
        timeinfo.tm_mon = month - 1;
        timeinfo.tm_mday = day;
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = minute;
        timeinfo.tm_sec = static_cast<int>(second);

        time_t sim_time = std::mktime(&timeinfo);
        if (sim_time == -1) {
            throw std::runtime_error("Invalid simulation time specified");
        }

        set_simulation_start_time(sim_time);
    }
    catch (const std::exception& e) {
        std::cerr << "Error setting simulation time: " << e.what() << std::endl;
        throw;
    }
}

void doppler_simulator_sgp4_impl::set_simulation_start_time(time_t unix_time)
{
    simulation_start_time_ = unix_time;
    simulation_offset_ = simulation_start_time_ - std::time(nullptr);
    current_time_ = 0;

    std::tm* timeinfo = std::localtime(&simulation_start_time_);
    std::cerr << "Set simulation start time to: "
              << std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S")
              << std::endl;

    update_position(simulation_start_time_);
}

void doppler_simulator_sgp4_impl::set_realtime_mode(bool enable)
{
    realtime_mode_ = enable;
    if (enable) {
        simulation_offset_ = 0;
        simulation_start_time_ = std::time(nullptr);
        current_time_ = 0;
        std::cerr << "Switched to realtime mode" << std::endl;
    }
}

time_t doppler_simulator_sgp4_impl::get_simulation_time() const
{
    if (realtime_mode_) {
        return std::time(nullptr);
    } else {
        return simulation_start_time_ + static_cast<time_t>(current_time_);
    }
}

void doppler_simulator_sgp4_impl::update_position(double unix_time)
{
    try {
        auto pos = orbit_->calculate_position(unix_time);

        current_elevation_ = pos.elevation;
        current_azimuth_ = pos.azimuth;
        current_range_ = pos.range;
        current_velocity_ = pos.velocity;

        constexpr double LIGHT_SPEED = 299792.458;  // km/s
        current_doppler_ = 1.0f - (current_velocity_ / LIGHT_SPEED);

        float ant_gain = antenna_.calculate_gain(current_azimuth_, current_elevation_);


        double max_range = orbit_->MAX_RANGE;
        double normalized_range = std::max(static_cast<double>(current_range_), orbit_->MIN_RANGE) / max_range;
        double distance_loss = 1.0 / (normalized_range * normalized_range);

        double atmospheric_loss = 1.0;
        if (current_elevation_ < 20.0) {
            atmospheric_loss = std::pow(std::sin(current_elevation_ * M_PI / 180.0), 0.5);
        }

        float total_gain = ant_gain * distance_loss * atmospheric_loss;

        current_strength_ = std::max(0.0f, std::min(1.0f, static_cast<float>(total_gain)));

        if (current_elevation_ < 5.0 || current_strength_ < 0.05) {
            current_strength_ = 0.0f;
        }

        if (pos.velocity > 0) {
            current_heading_ = current_azimuth_ + 90.0f;
        } else {
            current_heading_ = current_azimuth_ - 90.0f;
        }
        if (current_heading_ >= 360.0f) current_heading_ -= 360.0f;
        if (current_heading_ < 0.0f) current_heading_ += 360.0f;

        static int debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            std::cerr << "Satellite status:"
                      << "\n  Elevation: " << current_elevation_ << "°"
                      << "\n  Azimuth: " << current_azimuth_ << "°"
                      << "\n  Range: " << current_range_ << " km"
                      << "\n  Antenna gain: " << ant_gain
                      << "\n  Distance loss: " << distance_loss
                      << "\n  Atmospheric loss: " << atmospheric_loss
                      << "\n  Total strength: " << current_strength_
                      << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error updating position: " << e.what() << std::endl;
        throw;
    }
}

void doppler_simulator_sgp4_impl::publish_status()
{
    try {
        auto pos = orbit_->calculate_position(get_simulation_time());
        
        pmt::pmt_t dict = pmt::make_dict();
        
        // Current satellite position
        dict = pmt::dict_add(dict, pmt::intern("latitude"), pmt::from_double(pos.latitude));
        dict = pmt::dict_add(dict, pmt::intern("longitude"), pmt::from_double(pos.longitude));
        
        // Ground station information
        dict = pmt::dict_add(dict, pmt::intern("antennaLat"), 
            pmt::from_double(orbit_->get_ground_station().latitude));
        dict = pmt::dict_add(dict, pmt::intern("antennaLon"), 
            pmt::from_double(orbit_->get_ground_station().longitude));
        dict = pmt::dict_add(dict, pmt::intern("antennaAlt"), 
            pmt::from_double(orbit_->get_ground_station().altitude));
        dict = pmt::dict_add(dict, pmt::intern("antennaAzimuth"), 
            pmt::from_double(antenna_.get_azimuth_offset()));
        dict = pmt::dict_add(dict, pmt::intern("antennaElevation"), 
            pmt::from_double(antenna_.get_elevation_offset()));
        
        // Signal parameters
        dict = pmt::dict_add(dict, pmt::intern("elevation"), pmt::from_double(current_elevation_));
        dict = pmt::dict_add(dict, pmt::intern("azimuth"), pmt::from_double(current_azimuth_));
        dict = pmt::dict_add(dict, pmt::intern("range"), pmt::from_double(current_range_));
        dict = pmt::dict_add(dict, pmt::intern("velocity"), pmt::from_double(current_velocity_));
        dict = pmt::dict_add(dict, pmt::intern("doppler"), pmt::from_double(current_doppler_));
        dict = pmt::dict_add(dict, pmt::intern("strength"), pmt::from_double(current_strength_));
        dict = pmt::dict_add(dict, pmt::intern("heading"), pmt::from_double(current_heading_));
        
        // Simulation time
        dict = pmt::dict_add(dict, pmt::intern("simulationTime"), 
            pmt::from_double(get_simulation_time()));

        message_port_pub(pmt::intern("sat_pos"), dict);
    }
    catch (const std::exception& e) {
        std::cerr << "Error publishing status: " << e.what() << std::endl;
    }
}

} // namespace apt_encoder
} // namespace gr