#ifndef INCLUDED_APT_ENCODER_IMAGE_HANDLER_HPP
#define INCLUDED_APT_ENCODER_IMAGE_HANDLER_HPP

#include <cstdint>
#include <string>
#include <memory>
#include <vector>

namespace gr {
namespace apt_encoder {

struct RgbColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct HsvColor {
    uint8_t h;
    uint8_t s;
    uint8_t v;
};

struct AptColor {
    uint8_t h;  // Hue
    uint8_t sv; // Reduced saturation and value
};

class ImageHandler {
public:
    static const int IMG_TGA_HEAD_SIZE = 18;
    static const int APT_VIDEO_WIDTH = 909;  // APT video width in pixels

    struct TgaHeader {
        uint8_t  id_length;
        uint8_t  color_map_type;
        uint8_t  image_type;
        uint16_t color_map_start;
        uint16_t color_map_length;
        uint8_t  color_map_depth;
        uint16_t x_offset;
        uint16_t y_offset;
        uint16_t width;
        uint16_t height;
        uint8_t  pixel_depth;
        uint8_t  image_descriptor;
    };

    ImageHandler(const std::string& filename);
    ~ImageHandler();

    // Prevent copying
    ImageHandler(const ImageHandler&) = delete;
    ImageHandler& operator=(const ImageHandler&) = delete;

    // File operations
    bool open();
    void close();
    bool is_open() const { return file_ != nullptr; }
    
    // Image properties
    uint16_t get_width() const { return header_.width; }
    uint16_t get_height() const { return header_.height; }
    uint8_t get_pixel_depth() const { return header_.pixel_depth; }
    void print_image_info() const; 

    // Pixel operations
    bool read_pixel(RgbColor& color);
    bool read_line(std::vector<RgbColor>& line_buffer);
    void seek_line(uint16_t line);

    // Color space conversions
    static HsvColor rgb_to_hsv(const RgbColor& rgb);
    static RgbColor hsv_to_rgb(const HsvColor& hsv);
    static AptColor rgb_to_apt(const RgbColor& rgb);
    static RgbColor apt_to_rgb(const AptColor& apt);

private:
    std::string filename_;
    FILE* file_;
    TgaHeader header_;
    bool read_header();
    bool validate_header();
    
    // Helper method for reading little-endian integers
    template<typename T>
    bool read_le(T& value);
};

} // namespace apt_encoder
} // namespace gr

#endif // INCLUDED_APT_ENCODER_IMAGE_HANDLER_HPP