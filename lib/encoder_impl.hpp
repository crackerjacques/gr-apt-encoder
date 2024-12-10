#ifndef INCLUDED_APT_ENCODER_IMPL_HPP
#define INCLUDED_APT_ENCODER_IMPL_HPP

#include <gnuradio/apt_encoder/encoder.hpp>
#include "image_handler.hpp"
#include "apt_generator.hpp"
#include "apt_format.hpp"
#include "apt_timing.hpp"
#include <memory>

namespace gr {
namespace apt_encoder {

class encoder_impl : public encoder
{
private:
    // Image processing
    std::unique_ptr<ImageHandler> d_primary_image;
    std::unique_ptr<ImageHandler> d_secondary_image;
    std::vector<uint8_t> d_video_buffer_a;
    std::vector<uint8_t> d_video_buffer_b;

    // APT signal generation
    std::unique_ptr<AptGenerator> d_generator;
    AptTelemetry d_telemetry_a;
    AptTelemetry d_telemetry_b;
    
    // Configuration
    double d_sample_rate;
    double d_carrier_freq;
    bool d_loop;
    char d_mode;

    // State tracking
    uint16_t d_current_line;
    uint8_t d_frame_counter;
    
    // Output buffer management
    std::vector<float> d_output_buffer;
    size_t d_samples_per_line;
    size_t d_buffer_offset;

    // Internal methods
    bool init_images(const std::string& primary_file, const std::string& secondary_file);
    void process_image_line();
    void generate_apt_line();
    void init_telemetry();

    // Debug helpers
    void print_debug_info(const char* stage) const;
    static uint64_t call_count;

    //save image file
    std::string d_primary_file;
    std::string d_secondary_file;

    const pmt::pmt_t d_port_duration;
    const pmt::pmt_t d_port_samples;

    //orbital 

    static constexpr double FIRST_COSMIC_VELOCITY = 7.8;  // km/s
    static constexpr double EARTH_RADIUS = 6371.0;        // km
    static constexpr double NOAA_ORBIT_HEIGHT = 850.0;    // km

public:
    // Constructor and Destructor
    encoder_impl(const std::string& image_file,
                const std::string& second_file,
                double sample_rate,
                double carrier_frequency,
                bool loop,
                char mode);
    ~encoder_impl() = default;

    // GNU Radio scheduler interface
    void forecast(int noutput_items, gr_vector_int &ninput_items_required);
    int work(int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);

    // Override methods from base class
    void reset() override;
    double estimate_duration() const override;
    uint64_t estimate_samples() const override;
    double calculate_path_length() const override;  
};

} // namespace apt_encoder
} // namespace gr

#endif /* INCLUDED_APT_ENCODER_IMPL_HPP */