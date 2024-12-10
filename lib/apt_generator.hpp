#ifndef INCLUDED_APT_GENERATOR_HPP
#define INCLUDED_APT_GENERATOR_HPP

#include "apt_format.hpp"
#include <vector>
#include <cmath>

namespace gr {
namespace apt_encoder {

class AptGenerator {
public:
    static constexpr float CARRIER_LEVEL = 0.3f;
    static constexpr float MOD_DEPTH = 0.7f;
    
    static constexpr float APT_SAMPLES_PER_LINE = 2080.0f;
    static constexpr float APT_WORD_LEN = 1.0f/4160.0f;
    static constexpr float APT_WORD_MUL = 5.7696f;

    AptGenerator(float sample_rate = 24000.0f, float carrier_freq = 2400.0f);

    // Generate sync patterns
    void generate_sync_a(std::array<uint8_t, AptConstants::SYNC_A>& buffer);
    void generate_sync_b(std::array<uint8_t, AptConstants::SYNC_B>& buffer);

    // Generate markers
    void generate_marker_a(std::array<uint8_t, AptConstants::MARKER_A>& buffer, bool minute_mark);
    void generate_marker_b(std::array<uint8_t, AptConstants::MARKER_B>& buffer, bool minute_mark);

    // Generate telemetry data
    void generate_telemetry(std::array<uint8_t, AptConstants::TELEMETRY_A>& buffer,
                          const AptTelemetry& telemetry,
                          uint8_t frame_number);

    // Generate complete APT line
    AptLine generate_line(uint8_t frame_number,
                         uint8_t line_number,
                         const std::vector<uint8_t>& video_a,
                         const std::vector<uint8_t>& video_b,
                         const AptTelemetry& telemetry_a,
                         const AptTelemetry& telemetry_b);

    // Convert digital samples to analog waveform with proper AM modulation
    void modulate(const std::vector<uint8_t>& digital_samples,
                 std::vector<float>& analog_samples);

    // Get samples per line at current sample rate
    size_t get_samples_per_line() const {
        return static_cast<size_t>(APT_SAMPLES_PER_LINE * APT_WORD_MUL);
    }

private:
    float sample_rate_;
    float carrier_freq_;
    float phase_;
    float phase_increment_;

    // Helper methods
    float generate_carrier(uint8_t amplitude);
    void reset_phase();
    
    // Debug helper
    void print_debug_info(const std::vector<uint8_t>& digital_samples,
                         const std::vector<float>& analog_samples) const;
};

} // namespace apt_encoder
} // namespace gr

#endif /* INCLUDED_APT_GENERATOR_HPP */