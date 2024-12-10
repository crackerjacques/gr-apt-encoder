#include "apt_format.hpp"
#include <algorithm>

namespace gr {
namespace apt_encoder {

std::array<uint8_t, AptConstants::LINE_LENGTH> AptLine::to_array() const {
    std::array<uint8_t, AptConstants::LINE_LENGTH> result;
    size_t offset = 0;

    // Channel A
    std::copy(sync_a.begin(), sync_a.end(), result.begin() + offset);
    offset += sync_a.size();
    
    std::copy(marker_a.begin(), marker_a.end(), result.begin() + offset);
    offset += marker_a.size();
    
    std::copy(video_a.begin(), video_a.end(), result.begin() + offset);
    offset += video_a.size();
    
    std::copy(telemetry_a.begin(), telemetry_a.end(), result.begin() + offset);
    offset += telemetry_a.size();

    // Channel B
    std::copy(sync_b.begin(), sync_b.end(), result.begin() + offset);
    offset += sync_b.size();
    
    std::copy(marker_b.begin(), marker_b.end(), result.begin() + offset);
    offset += marker_b.size();
    
    std::copy(video_b.begin(), video_b.end(), result.begin() + offset);
    offset += video_b.size();
    
    std::copy(telemetry_b.begin(), telemetry_b.end(), result.begin() + offset);

    return result;
}

} // namespace apt_encoder
} // namespace gr