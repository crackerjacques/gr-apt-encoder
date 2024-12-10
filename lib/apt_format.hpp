#ifndef INCLUDED_APT_FORMAT_HPP
#define INCLUDED_APT_FORMAT_HPP

#include <cstdint>
#include <array>

namespace gr {
namespace apt_encoder {

// APT signal format constants
struct AptConstants {
    static const int SYNC_A = 39;        // Sync A pulse width
    static const int SYNC_B = 39;        // Sync B pulse width
    static const int MARKER_A = 47;      // Marker A width
    static const int MARKER_B = 47;      // Marker B width
    static const int VIDEO_A = 909;      // Video A width
    static const int VIDEO_B = 909;      // Video B width
    static const int TELEMETRY_A = 45;   // Telemetry A width
    static const int TELEMETRY_B = 45;   // Telemetry B width
    static const int LINE_LENGTH = 2080; // Total APT line length
    static const int FRAME_SIZE = 128;   // Number of lines in a frame
    static const int MARKER_SIZE = 120;  // Marker repeat interval

    // Signal levels
    static const uint8_t SYNC_LOW = 11;
    static const uint8_t SYNC_HIGH = 244;
    static const uint8_t MARKER_LOW = 0;
    static const uint8_t MARKER_HIGH = 255;
};

// Telemetry wedge values
struct AptTelemetry {
    uint8_t wedge1;
    uint8_t wedge2;
    uint8_t wedge3;
    uint8_t wedge4;
    uint8_t wedge5;
    uint8_t wedge6;
    uint8_t wedge7;
    uint8_t wedge8;
    uint8_t zero_mod_ref;
    uint8_t temp1;
    uint8_t temp2;
    uint8_t temp3;
    uint8_t temp4;
    uint8_t patch_temp;
    uint8_t back_scan;
    uint8_t channel_id;

    // Constructor with default values
    AptTelemetry() : 
        wedge1(31),      // 12%
        wedge2(63),      // 25%
        wedge3(95),      // 37%
        wedge4(127),     // 50%
        wedge5(159),     // 62%
        wedge6(191),     // 75%
        wedge7(223),     // 87%
        wedge8(255),     // 100%
        zero_mod_ref(0), 
        temp1(105),
        temp2(105),
        temp3(105),
        temp4(105),
        patch_temp(120),
        back_scan(150),
        channel_id(31)
    {}
};

// Complete APT line structure
struct AptLine {
    std::array<uint8_t, AptConstants::SYNC_A> sync_a;
    std::array<uint8_t, AptConstants::MARKER_A> marker_a;
    std::array<uint8_t, AptConstants::VIDEO_A> video_a;
    std::array<uint8_t, AptConstants::TELEMETRY_A> telemetry_a;
    std::array<uint8_t, AptConstants::SYNC_B> sync_b;
    std::array<uint8_t, AptConstants::MARKER_B> marker_b;
    std::array<uint8_t, AptConstants::VIDEO_B> video_b;
    std::array<uint8_t, AptConstants::TELEMETRY_B> telemetry_b;

    std::array<uint8_t, AptConstants::LINE_LENGTH> to_array() const;
};

// Channel identifiers
enum class AptChannelId : uint8_t {
    VISIBLE = 31,     // Channel 1: Visible
    NEAR_IR = 63,     // Channel 2: Near IR
    MID_IR = 95,      // Channel 3A: Mid IR
    THERMAL_1 = 127,  // Channel 3B: Thermal IR)
    THERMAL_2 = 159,  // Channel 4: Thermal IR
    THERMAL_3 = 191   // Channel 5: Thermal IR
};

} // namespace apt
} // namespace gr

#endif /* INCLUDED_APT_FORMAT_HPP */