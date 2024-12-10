#include "apt_generator.hpp"
#include <iostream>
#include <algorithm>

namespace gr {
namespace apt_encoder {

AptGenerator::AptGenerator(float sample_rate, float carrier_freq)
    : sample_rate_(sample_rate)
    , carrier_freq_(carrier_freq)
    , phase_(0.0f)
    , phase_increment_(2.0f * M_PI * carrier_freq / sample_rate)
{
    std::cerr << "APT timing check:"
              << "\n  Samples per line: " << get_samples_per_line()
              << "\n  Line duration: " << (get_samples_per_line() / sample_rate_)
              << " seconds"
              << "\n  Sample rate: " << sample_rate_
              << "\n  Carrier frequency: " << carrier_freq_
              << std::endl;
}

void AptGenerator::generate_sync_a(std::array<uint8_t, AptConstants::SYNC_A>& buffer)
{
    for (int i = 0; i < AptConstants::SYNC_A; i++) {
        if ((i >= 4 && i <= 5) || (i >= 8 && i <= 9) || 
            (i >= 12 && i <= 13) || (i >= 16 && i <= 17) ||
            (i >= 20 && i <= 21) || (i >= 24 && i <= 25) ||
            (i >= 28 && i <= 29)) {
            buffer[i] = AptConstants::SYNC_HIGH;
        } else {
            buffer[i] = AptConstants::SYNC_LOW;
        }
    }
}

void AptGenerator::generate_sync_b(std::array<uint8_t, AptConstants::SYNC_B>& buffer)
{
    for (int i = 0; i < AptConstants::SYNC_B; i++) {
        if ((i >= 4 && i <= 6) || (i >= 9 && i <= 11) ||
            (i >= 14 && i <= 16) || (i >= 19 && i <= 21) ||
            (i >= 24 && i <= 26) || (i >= 29 && i <= 31) ||
            (i >= 34 && i <= 36)) {
            buffer[i] = AptConstants::SYNC_HIGH;
        } else {
            buffer[i] = AptConstants::SYNC_LOW;
        }
    }
}

void AptGenerator::generate_marker_a(std::array<uint8_t, AptConstants::MARKER_A>& buffer, bool minute_mark)
{
    for (int i = 0; i < AptConstants::MARKER_A; i++) {
        buffer[i] = minute_mark ? AptConstants::MARKER_HIGH : AptConstants::MARKER_LOW;
    }
}

void AptGenerator::generate_marker_b(std::array<uint8_t, AptConstants::MARKER_B>& buffer, bool minute_mark)
{
    for (int i = 0; i < AptConstants::MARKER_B; i++) {
        buffer[i] = minute_mark ? AptConstants::MARKER_LOW : AptConstants::MARKER_HIGH;
    }
}

void AptGenerator::generate_telemetry(std::array<uint8_t, AptConstants::TELEMETRY_A>& buffer,
                                    const AptTelemetry& telemetry,
                                    uint8_t frame_number)
{
    frame_number = ((frame_number - 1) % AptConstants::FRAME_SIZE) + 1;

    // debug
    if (frame_number >= 33) {
        std::cerr << "Telemetry generation for frame " << static_cast<int>(frame_number)
                  << "\n  Buffer size: " << buffer.size()
                  << std::endl;
    }

    uint8_t telemetry_value;
    
    if (frame_number <= 8) telemetry_value = telemetry.wedge1;
    else if (frame_number <= 16) telemetry_value = telemetry.wedge2;
    else if (frame_number <= 24) telemetry_value = telemetry.wedge3;
    else if (frame_number <= 32) telemetry_value = telemetry.wedge4;
    else if (frame_number <= 40) telemetry_value = telemetry.wedge5;
    else if (frame_number <= 48) telemetry_value = telemetry.wedge6;
    else if (frame_number <= 56) telemetry_value = telemetry.wedge7;
    else if (frame_number <= 64) telemetry_value = telemetry.wedge8;
    else if (frame_number <= 72) telemetry_value = telemetry.zero_mod_ref;
    else if (frame_number <= 80) telemetry_value = telemetry.temp1;
    else if (frame_number <= 88) telemetry_value = telemetry.temp2;
    else if (frame_number <= 96) telemetry_value = telemetry.temp3;
    else if (frame_number <= 104) telemetry_value = telemetry.temp4;
    else if (frame_number <= 112) telemetry_value = telemetry.patch_temp;
    else if (frame_number <= 120) telemetry_value = telemetry.back_scan;
    else telemetry_value = telemetry.channel_id;

    std::fill(buffer.begin(), buffer.end(), telemetry_value);
}

AptLine AptGenerator::generate_line(uint8_t frame_number,
                                 uint8_t line_number,
                                 const std::vector<uint8_t>& video_a,
                                 const std::vector<uint8_t>& video_b,
                                 const AptTelemetry& telemetry_a,
                                 const AptTelemetry& telemetry_b)
{
    AptLine line;
    
    bool minute_mark = (line_number == 0 || line_number == 1);

    generate_sync_a(line.sync_a);
    generate_sync_b(line.sync_b);
    generate_marker_a(line.marker_a, minute_mark);
    generate_marker_b(line.marker_b, minute_mark);

    std::copy(video_a.begin(), video_a.end(), line.video_a.begin());
    std::copy(video_b.begin(), video_b.end(), line.video_b.begin());

    generate_telemetry(line.telemetry_a, telemetry_a, frame_number);
    generate_telemetry(line.telemetry_b, telemetry_b, frame_number);

    return line;
}

float AptGenerator::generate_carrier(uint8_t amplitude)
{
    float normalized_amplitude = static_cast<float>(amplitude) / 255.0f;
    float carrier = std::sin(phase_);
    
    static uint64_t sample_count = 0;
    if (++sample_count % 24000 == 0) { 
        std::cerr << "Carrier sample:"
                  << "\n  Amplitude: " << static_cast<int>(amplitude) 
                  << "\n  Normalized: " << normalized_amplitude
                  << "\n  Carrier: " << carrier
                  << "\n  Phase: " << phase_
                  << "\n  Output: " << (CARRIER_LEVEL + (MOD_DEPTH * normalized_amplitude * carrier))
                  << std::endl;
    }
    
    float modulated = CARRIER_LEVEL + (MOD_DEPTH * normalized_amplitude * carrier);
    
    phase_ += phase_increment_;
    if (phase_ >= 2.0f * M_PI) {
        phase_ -= 2.0f * M_PI;
    }

    return modulated;
}

void AptGenerator::modulate(const std::vector<uint8_t>& digital_samples,
                         std::vector<float>& analog_samples)
{
    size_t output_size = static_cast<size_t>(digital_samples.size() * APT_WORD_MUL);
    analog_samples.resize(output_size);
    reset_phase();
    
    for (size_t i = 0; i < output_size; i++) {
        size_t input_idx = static_cast<size_t>(i / APT_WORD_MUL);
        if (input_idx >= digital_samples.size()) {
            input_idx = digital_samples.size() - 1;
        }
        analog_samples[i] = generate_carrier(digital_samples[input_idx]);
    }

    static uint64_t call_count = 0;
    if (++call_count % 100 == 0) {
        print_debug_info(digital_samples, analog_samples);
    }
}

void AptGenerator::reset_phase()
{
    phase_ = 0.0f;
}

void AptGenerator::print_debug_info(const std::vector<uint8_t>& digital_samples,
                                  const std::vector<float>& analog_samples) const
{
    float min_sample = *std::min_element(analog_samples.begin(), analog_samples.end());
    float max_sample = *std::max_element(analog_samples.begin(), analog_samples.end());
    
    uint8_t min_digital = *std::min_element(digital_samples.begin(), digital_samples.end());
    uint8_t max_digital = *std::max_element(digital_samples.begin(), digital_samples.end());

    std::cerr << "APT Signal Stats:"
              << "\n  Digital range: " << static_cast<int>(min_digital) 
              << " to " << static_cast<int>(max_digital)
              << "\n  Analog range: " << min_sample << " to " << max_sample
              << "\n  Digital samples: " << digital_samples.size()
              << "\n  Analog samples: " << analog_samples.size()
              << "\n  Samples ratio: " << (float)analog_samples.size() / digital_samples.size()
              << std::endl;
}

} // namespace apt_encoder
} // namespace gr