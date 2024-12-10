#ifndef INCLUDED_APT_TIMING_HPP
#define INCLUDED_APT_TIMING_HPP

#include "apt_generator.hpp"
#include "image_handler.hpp"
#include <string>

namespace gr {
namespace apt_encoder {

class AptTiming {
public:
    // Calculate duration in seconds based on image file
    static double estimate_duration(const std::string& image_file, float sample_rate = 24000.0f) {
        ImageHandler image(image_file);
        if (!image.open()) {
            return 0.0;
        }
        
        uint16_t image_height = image.get_height();
        image.close();
        
        return estimate_duration_from_lines(image_height, sample_rate);
    }
    
    // Calculate duration in seconds based on number of lines
    static double estimate_duration_from_lines(uint16_t num_lines, float sample_rate = 24000.0f) {
        // Calculate samples per line (from apt_generator.hpp)
        double samples_per_line = AptGenerator::APT_SAMPLES_PER_LINE * AptGenerator::APT_WORD_MUL;
        
        // Calculate total samples
        double total_samples = samples_per_line * num_lines;
        
        // Convert to duration
        return total_samples / sample_rate;
    }
    
    // Calculate number of samples that will be produced
    static uint64_t estimate_samples(const std::string& image_file, float sample_rate = 24000.0f) {
        ImageHandler image(image_file);
        if (!image.open()) {
            return 0;
        }
        
        uint16_t image_height = image.get_height();
        image.close();
        
        return estimate_samples_from_lines(image_height, sample_rate);
    }
    
    // Calculate number of samples based on number of lines
    static uint64_t estimate_samples_from_lines(uint16_t num_lines, float sample_rate = 24000.0f) {
        double samples_per_line = AptGenerator::APT_SAMPLES_PER_LINE * AptGenerator::APT_WORD_MUL;
        return static_cast<uint64_t>(samples_per_line * num_lines);
    }
};

} // namespace apt_encoder
} // namespace gr

#endif /* INCLUDED_APT_TIMING_HPP */
