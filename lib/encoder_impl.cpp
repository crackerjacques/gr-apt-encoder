#include "encoder_impl.hpp"
#include <gnuradio/io_signature.h>
#include <iostream>

namespace gr {
namespace apt_encoder {

uint64_t encoder_impl::call_count = 0;

encoder::sptr encoder::make(const std::string& image_file,
                          const std::string& second_file,
                          double sample_rate,
                          double carrier_frequency,
                          bool loop,
                          char mode)
{
    return gnuradio::make_block_sptr<encoder_impl>(
        image_file, second_file, sample_rate, carrier_frequency, loop, mode);
}

encoder_impl::encoder_impl(const std::string& image_file,
                        const std::string& second_file,
                        double sample_rate,
                        double carrier_frequency,
                        bool loop,
                        char mode)
   : gr::sync_block("encoder",
                    gr::io_signature::make(0, 0, 0),
                    gr::io_signature::make(1, 1, sizeof(float))),
     d_sample_rate(sample_rate),
     d_carrier_freq(carrier_frequency),
     d_loop(loop),
     d_mode(mode),
     d_current_line(0),
     d_frame_counter(1),
     d_buffer_offset(0),
     d_primary_file(image_file),
     d_secondary_file(second_file)
{
   try {
       message_port_register_out(pmt::intern("duration"));
       message_port_register_out(pmt::intern("samples"));
       message_port_register_out(pmt::intern("path_length"));

       // Initialize image processors
       if (!init_images(d_primary_file, d_secondary_file)) {
           throw std::runtime_error("Failed to initialize image files");
       }

       // Initialize APT generator
       d_generator = std::make_unique<AptGenerator>(sample_rate, carrier_frequency);

       // Initialize buffers
       d_video_buffer_a.resize(AptConstants::VIDEO_A);
       d_video_buffer_b.resize(AptConstants::VIDEO_B);

       d_samples_per_line = d_generator->get_samples_per_line();
       d_output_buffer.resize(d_samples_per_line);

       // Initialize telemetry data
       init_telemetry();

       // Set output buffer size hint
       set_output_multiple(d_samples_per_line);

       // initial messages
       double duration = estimate_duration();
       uint64_t samples = estimate_samples();
       double path_length = calculate_path_length();

       message_port_pub(pmt::intern("duration"), pmt::from_double(duration));
       message_port_pub(pmt::intern("samples"), pmt::from_uint64(samples));
       message_port_pub(pmt::intern("path_length"), pmt::from_double(path_length));

       std::cerr << "Encoder initialization:"
                 << "\n  Sample rate: " << d_sample_rate
                 << "\n  Samples per line: " << d_samples_per_line
                 << "\n  Buffer size: " << d_output_buffer.size()
                 << "\n  Primary file: " << d_primary_file
                 << "\n  Secondary file: " << (d_secondary_file.empty() ? "none" : d_secondary_file)
                 << "\n  Estimated duration: " << duration << " seconds"
                 << "\n  Estimated path length: " << path_length << " km"
                 << std::endl;
   }
   catch (const std::exception& e) {
       std::cerr << "Error during encoder initialization: " << e.what() << std::endl;
       throw;
   }
}

void encoder_impl::reset()
{
    try {
        d_current_line = 0;
        d_frame_counter = 1;
        d_buffer_offset = 0;

        std::fill(d_output_buffer.begin(), d_output_buffer.end(), 0.0f);
        std::fill(d_video_buffer_a.begin(), d_video_buffer_a.end(), 0);
        std::fill(d_video_buffer_b.begin(), d_video_buffer_b.end(), 0);

        if (d_primary_image) {
            d_primary_image->close();
        }
        if (d_secondary_image) {
            d_secondary_image->close();
        }

        if (!init_images(d_primary_file, d_secondary_file)) {
            throw std::runtime_error("Failed to reinitialize image files during reset");
        }

        d_generator = std::make_unique<AptGenerator>(d_sample_rate, d_carrier_freq);

        std::cerr << "Encoder reset completed successfully" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error during encoder reset: " << e.what() << std::endl;
        throw;
    }
}


bool encoder_impl::init_images(const std::string& primary_file, const std::string& secondary_file)
{
    d_primary_image = std::make_unique<ImageHandler>(primary_file);
    if (!d_primary_image->open()) {
        std::cerr << "Failed to open primary image: " << primary_file << std::endl;
        return false;
    }

    if (!secondary_file.empty()) {
        d_secondary_image = std::make_unique<ImageHandler>(secondary_file);
        if (!d_secondary_image->open()) {
            std::cerr << "Failed to open secondary image: " << secondary_file << std::endl;
            return false;
        }
    }

    return true;
}

void encoder_impl::init_telemetry()
{
    // Channel A telemetry (visible channel)
    d_telemetry_a = AptTelemetry();
    d_telemetry_a.channel_id = static_cast<uint8_t>(AptChannelId::VISIBLE);

    // Channel B telemetry (IR channel or based on mode)
    d_telemetry_b = AptTelemetry();
    switch (d_mode) {
        case 'R':
        case 'G':
        case 'B':
            d_telemetry_b.channel_id = static_cast<uint8_t>(AptChannelId::VISIBLE);
            break;
        case 'Y':
            d_telemetry_b.channel_id = static_cast<uint8_t>(AptChannelId::NEAR_IR);
            break;
        case 'C':
            d_telemetry_b.channel_id = static_cast<uint8_t>(AptChannelId::THERMAL_1);
            break;
        default:
            d_telemetry_b.channel_id = static_cast<uint8_t>(AptChannelId::VISIBLE);
    }
}

double encoder_impl::estimate_duration() const
{
    return AptTiming::estimate_duration(d_primary_file, d_sample_rate);
}

uint64_t encoder_impl::estimate_samples() const
{
    return AptTiming::estimate_samples(d_primary_file, d_sample_rate);
}

void encoder_impl::process_image_line()
{
    std::vector<RgbColor> line_buffer_a;
    std::vector<RgbColor> line_buffer_b;
    
    if (d_primary_image->read_line(line_buffer_a)) {
        for (size_t i = 0; i < AptConstants::VIDEO_A; i++) {
            d_video_buffer_a[i] = static_cast<uint8_t>(
                line_buffer_a[i].r * 0.299 + 
                line_buffer_a[i].g * 0.587 + 
                line_buffer_a[i].b * 0.114);
        }
    }

    if (d_secondary_image) {
        if (d_secondary_image->read_line(line_buffer_b)) {
            for (size_t i = 0; i < AptConstants::VIDEO_B; i++) {
                d_video_buffer_b[i] = static_cast<uint8_t>(
                    line_buffer_b[i].r * 0.299 + 
                    line_buffer_b[i].g * 0.587 + 
                    line_buffer_b[i].b * 0.114);
            }
        }
    } else {
        for (size_t i = 0; i < AptConstants::VIDEO_B; i++) {
            const auto& pixel = line_buffer_a[i];
            switch (d_mode) {
                case 'R': d_video_buffer_b[i] = pixel.r; break;
                case 'G': d_video_buffer_b[i] = pixel.g; break;
                case 'B': d_video_buffer_b[i] = pixel.b; break;
                case 'N': d_video_buffer_b[i] = 255 - d_video_buffer_a[i]; break;
                case 'Y': 
                    d_video_buffer_b[i] = static_cast<uint8_t>(
                        (-0.148 * pixel.r - 0.291 * pixel.g + 0.439 * pixel.b + 128));
                    break;
                case 'C': {
                    AptColor apt = ImageHandler::rgb_to_apt(pixel);
                    d_video_buffer_a[i] = apt.h;
                    d_video_buffer_b[i] = apt.sv;
                    break;
                }
            }
        }
    }
}

void encoder_impl::generate_apt_line()
{
    // if (d_current_line == 33) {
    //     std::cerr << "Line 33 detailed debug:"
    //               << "\n  Current line: " << d_current_line
    //               << "\n  Frame counter: " << static_cast<int>(d_frame_counter)
    //               << "\n  Buffer offset: " << d_buffer_offset
    //               << "\n  Image height: " << d_primary_image->get_height()
    //               << std::endl;
    // }

    // Generate complete APT line
    AptLine line = d_generator->generate_line(
        d_frame_counter,
        d_current_line % AptConstants::MARKER_SIZE,
        d_video_buffer_a,
        d_video_buffer_b,
        d_telemetry_a,
        d_telemetry_b
    );

    // Convert line to array and then to vector for modulation
    auto digital_line = line.to_array();
    std::vector<uint8_t> vector_line(digital_line.begin(), digital_line.end());

    if (d_output_buffer.size() != d_samples_per_line) {
        std::cerr << "Buffer size mismatch:"
                  << "\n  Expected: " << d_samples_per_line
                  << "\n  Actual: " << d_output_buffer.size()
                  << std::endl;
        d_output_buffer.resize(d_samples_per_line);
    }

    try {
        d_generator->modulate(vector_line, d_output_buffer);
    } catch (const std::exception& e) {
        std::cerr << "Exception in modulate: " << e.what() << std::endl;
    }

    std::cerr << "Line generation complete:"
              << "\n  Digital line size: " << digital_line.size()
              << "\n  Expected output size: " << d_samples_per_line
              << "\n  Frame: " << static_cast<int>(d_frame_counter)
              << "\n  Line: " << d_current_line
              << "\n  Output buffer size: " << d_output_buffer.size()
              << std::endl;
}

void encoder_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
{
    // No inputs required
    return;
}

double encoder_impl::calculate_path_length() const
{
    double orbit_radius = EARTH_RADIUS + NOAA_ORBIT_HEIGHT;
    double time_s = estimate_duration();    
    double distance = FIRST_COSMIC_VELOCITY * time_s;    
    double ground_path = (distance * EARTH_RADIUS) / orbit_radius;
    
    return ground_path;
}

int encoder_impl::work(int noutput_items,
                      gr_vector_const_void_star &input_items,
                      gr_vector_void_star &output_items)
{
    static bool first_run = true;
    if (first_run) {
        first_run = false;
        

        double duration = estimate_duration();
        pmt::pmt_t duration_msg = pmt::from_double(duration);
        message_port_pub(pmt::intern("duration"), duration_msg);

        uint64_t samples = estimate_samples();
        pmt::pmt_t samples_msg = pmt::from_uint64(samples);
        message_port_pub(pmt::intern("samples"), samples_msg);

        double path_length = calculate_path_length();
        pmt::pmt_t path_msg = pmt::from_double(path_length);
        message_port_pub(pmt::intern("path_length"), path_msg);
        
        std::cerr << "APT Encoder estimates:" 
                  << "\n  Duration: " << duration << " seconds"
                  << "\n  Total samples: " << samples
                  << "\n  Path length: " << path_length << " km"
                  << std::endl;
    }

    float *out = static_cast<float*>(output_items[0]);
    int produced = 0;

    while (produced < noutput_items) {
        if (d_buffer_offset == 0) {
            if (d_current_line >= d_primary_image->get_height()) {
                if (produced > 0) {
                    float fade_factor = 0.995f;
                    for (int i = 0; i < produced; i++) {
                        out[i] *= fade_factor;
                        fade_factor *= 0.995f;
                    }
                }
                return produced > 0 ? produced : WORK_DONE;
            }

            if (d_frame_counter > AptConstants::FRAME_SIZE) {
                d_frame_counter = 1;
            }

            process_image_line();
            generate_apt_line();
            d_current_line++;
            d_frame_counter++;
        }

        size_t samples_to_copy = std::min(
            static_cast<size_t>(noutput_items - produced),
            d_samples_per_line - d_buffer_offset
        );

        std::memcpy(&out[produced],
                   &d_output_buffer[d_buffer_offset],
                   samples_to_copy * sizeof(float));

        produced += samples_to_copy;
        d_buffer_offset += samples_to_copy;

        if (d_buffer_offset >= d_samples_per_line) {
            d_buffer_offset = 0;
        }
    }

    return produced;
}

} // namespace apt_encoder
} // namespace gr