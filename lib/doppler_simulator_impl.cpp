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
                                             float trim_threshold)
{
   return gnuradio::make_block_sptr<doppler_simulator_impl>(
       sample_rate, center_freq, range_km, sat_velocity, auto_trim, trim_threshold);
}

doppler_simulator_impl::doppler_simulator_impl(float sample_rate,
                                            float center_freq,
                                            float range_km,
                                            float sat_velocity,
                                            bool auto_trim,
                                            float trim_threshold)
   : gr::sync_block("doppler_simulator",
                    gr::io_signature::make(1, 1, sizeof(float)),
                    gr::io_signature::make(1, 1, sizeof(float))),
     d_sample_rate(sample_rate),
     d_center_freq(center_freq),
     d_range_km(range_km),
     d_sat_velocity(sat_velocity),
     d_auto_trim(auto_trim),
     d_trim_threshold(trim_threshold),
     d_duration(0),
     d_total_samples(0),
     d_path_length(0),
     d_params_set(false),
     d_processed_samples(0),
     d_current_doppler(1.0),
     d_current_strength(1.0)
{
   message_port_register_in(pmt::intern("duration"));
   message_port_register_in(pmt::intern("samples"));
   message_port_register_in(pmt::intern("path_length"));
   message_port_register_out(pmt::intern("sat_pos"));

   set_msg_handler(pmt::intern("duration"),
       [this](const pmt::pmt_t& msg) { this->handle_duration_msg(msg); });
   set_msg_handler(pmt::intern("samples"),
       [this](const pmt::pmt_t& msg) { this->handle_samples_msg(msg); });
   set_msg_handler(pmt::intern("path_length"),
       [this](const pmt::pmt_t& msg) { this->handle_path_length_msg(msg); });

   std::cerr << "Doppler simulator initialized:"
             << "\n  Sample rate: " << d_sample_rate
             << "\n  Center frequency: " << d_center_freq
             << "\n  Range: " << d_range_km << " km"
             << "\n  Satellite velocity: " << d_sat_velocity << " km/s"
             << "\n  Auto trim: " << (d_auto_trim ? "enabled" : "disabled")
             << "\n  Trim threshold: " << d_trim_threshold << " degrees"
             << std::endl;
}

void doppler_simulator_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
   ninput_items_required[0] = noutput_items;
}

void doppler_simulator_impl::handle_duration_msg(pmt::pmt_t msg)
{
   if (pmt::is_number(msg)) {
       d_duration = pmt::to_double(msg);
       std::cerr << "Doppler simulator received duration: " 
                 << d_duration << " seconds" << std::endl;
   }
}

void doppler_simulator_impl::handle_samples_msg(pmt::pmt_t msg)
{
   if (pmt::is_uint64(msg)) {
       d_total_samples = pmt::to_uint64(msg);
       d_params_set = true;
       std::cerr << "Doppler simulator received total samples: " 
                 << d_total_samples << std::endl;
   }
}

void doppler_simulator_impl::handle_path_length_msg(pmt::pmt_t msg)
{
   if (pmt::is_number(msg)) {
       d_path_length = pmt::to_double(msg);
       if (d_auto_trim) {
           d_range_km = d_path_length / 2.0;
       }
       std::cerr << "Doppler simulator received path length: " 
                 << d_path_length << " km" << std::endl;
   }
}

double doppler_simulator_impl::calculate_elevation(double t) const
{
   double normalized = 2.0 * t - 1.0;  // -1 to 1
   double elev = 90.0 * (1.0 - normalized * normalized);
   return elev;
}

double doppler_simulator_impl::calculate_doppler_shift(double t) const
{
   double elevation_rad = calculate_elevation(t) * PI / 180.0;
   
   double los_velocity = d_sat_velocity * std::cos(elevation_rad);
   
   return 1.0 + (los_velocity / LIGHT_SPEED);
}

double doppler_simulator_impl::calculate_signal_strength(double elevation) const
{
   if (elevation < d_trim_threshold) return 0.0;
   
   return std::cos((90.0 - elevation) * PI / 180.0);
}

void doppler_simulator_impl::update_status(double t)
{
   double elevation = calculate_elevation(t);
   d_current_doppler = calculate_doppler_shift(t);
   d_current_strength = calculate_signal_strength(elevation);

   pmt::pmt_t pos_dict = pmt::make_dict();
   pos_dict = pmt::dict_add(pos_dict, pmt::intern("time"), pmt::from_double(t));
   pos_dict = pmt::dict_add(pos_dict, pmt::intern("elevation"), pmt::from_double(elevation));
   pos_dict = pmt::dict_add(pos_dict, pmt::intern("doppler"), pmt::from_double(d_current_doppler));
   pos_dict = pmt::dict_add(pos_dict, pmt::intern("strength"), pmt::from_double(d_current_strength));
   
   message_port_pub(pmt::intern("sat_pos"), pos_dict);
}

int doppler_simulator_impl::work(int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items)
{
   const float *in = (const float *) input_items[0];
   float *out = (float *) output_items[0];

   if (!d_params_set) {
       std::memcpy(out, in, noutput_items * sizeof(float));
       return noutput_items;
   }

   static float dc_sum = 0.0f;
   static const int DC_WINDOW = 1024;
   static std::vector<float> dc_buffer(DC_WINDOW, 0.0f);
   static int dc_index = 0;
   
   static float prev_strength = 1.0f;
   static float prev_doppler = 1.0f;

   for (int i = 0; i < noutput_items; i++) {
       double t = static_cast<double>(d_processed_samples + i) / d_total_samples;
       
       double elevation = calculate_elevation(t);
       double target_strength = calculate_signal_strength(elevation);
       double target_doppler = calculate_doppler_shift(t);
       
       float strength = prev_strength * 0.9f + target_strength * 0.1f;
       float doppler = prev_doppler * 0.9f + target_doppler * 0.1f;
       
       prev_strength = strength;
       prev_doppler = doppler;
       
       // DC Offset
       dc_sum -= dc_buffer[dc_index];
       dc_buffer[dc_index] = in[i];
       dc_sum += dc_buffer[dc_index];
       dc_index = (dc_index + 1) % DC_WINDOW;
       float dc_offset = dc_sum / DC_WINDOW;
       
       // final signal
       float signal = (in[i] - dc_offset) * strength * doppler;
       out[i] = std::max(-1.0f, std::min(1.0f, signal));

       // refresh
      if ((d_processed_samples + i) % static_cast<uint64_t>(d_sample_rate / 10) == 0) {
         update_status(t);
      }
   }
   
   d_processed_samples += noutput_items;
   
   if (d_processed_samples % (d_total_samples / 10) < static_cast<uint64_t>(noutput_items)) {
      std::cerr << "Doppler simulation progress: " 
               << (d_processed_samples * 100 / d_total_samples) << "%"
               << std::endl;
   }

   return noutput_items;
}

void doppler_simulator_impl::set_range(float range_km)
{
   d_range_km = range_km;
}

void doppler_simulator_impl::set_velocity(float velocity)
{
   d_sat_velocity = velocity;
}

void doppler_simulator_impl::set_auto_trim(bool enable)
{
   d_auto_trim = enable;
}

} // namespace apt_encoder
} // namespace gr