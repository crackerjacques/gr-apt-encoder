#include "image_handler.hpp"
#include <stdexcept>
#include <cstring>
#include <cmath>
#include <iostream> 

namespace gr {
namespace apt_encoder {

ImageHandler::ImageHandler(const std::string& filename)
    : filename_(filename)
    , file_(nullptr)
{
    std::memset(&header_, 0, sizeof(TgaHeader));
}

ImageHandler::~ImageHandler()
{
    close();
}

template<typename T>
bool ImageHandler::read_le(T& value)
{
    uint8_t buffer[sizeof(T)];
    if (fread(buffer, sizeof(buffer), 1, file_) != 1) {
        return false;
    }
    
    value = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
        value |= static_cast<T>(buffer[i]) << (8 * i);
    }
    return true;
}

bool ImageHandler::read_header()
{
    if (!file_) return false;

    // Read TGA header fields
    uint8_t byte;
    if (fread(&byte, 1, 1, file_) != 1) return false;
    header_.id_length = byte;
    
    if (fread(&byte, 1, 1, file_) != 1) return false;
    header_.color_map_type = byte;
    
    if (fread(&byte, 1, 1, file_) != 1) return false;
    header_.image_type = byte;
    
    if (!read_le(header_.color_map_start)) return false;
    if (!read_le(header_.color_map_length)) return false;
    
    if (fread(&byte, 1, 1, file_) != 1) return false;
    header_.color_map_depth = byte;
    
    if (!read_le(header_.x_offset)) return false;
    if (!read_le(header_.y_offset)) return false;
    if (!read_le(header_.width)) return false;
    if (!read_le(header_.height)) return false;
    
    if (fread(&byte, 1, 1, file_) != 1) return false;
    header_.pixel_depth = byte;
    
    if (fread(&byte, 1, 1, file_) != 1) return false;
    header_.image_descriptor = byte;

    return true;
}

void ImageHandler::print_image_info() const
{
    std::cerr << "Image info for: " << filename_ << std::endl
              << "  Width: " << header_.width << std::endl
              << "  Height: " << header_.height << std::endl
              << "  Pixel depth: " << (int)header_.pixel_depth << std::endl
              << "  Image type: " << (int)header_.image_type << std::endl;
}

bool ImageHandler::validate_header()
{
    // Verify image format requirements for APT
    if (header_.width != APT_VIDEO_WIDTH) {
        return false;
    }
    
    // We expect uncompressed true-color image
    if (header_.image_type != 2) {
        return false;
    }
    
    // We expect 24-bit RGB
    if (header_.pixel_depth != 24) {
        return false;
    }

    return true;
}



bool ImageHandler::open()
{
    close();
    
    file_ = fopen(filename_.c_str(), "rb");
    if (!file_) {
        return false;
    }

    if (!read_header() || !validate_header()) {
        close();
        return false;
    }

    return true;
}

void ImageHandler::close()
{
    if (file_) {
        fclose(file_);
        file_ = nullptr;
    }
}

bool ImageHandler::read_pixel(RgbColor& color)
{
    if (!file_) return false;

    uint8_t bgr[3];
    if (fread(bgr, 1, 3, file_) != 3) {
        return false;
    }

    // TGA stores in BGR format
    color.b = bgr[0];
    color.g = bgr[1];
    color.r = bgr[2];

    return true;
}

bool ImageHandler::read_line(std::vector<RgbColor>& line_buffer)
{
    if (!file_) return false;

    line_buffer.resize(header_.width);
    for (uint16_t i = 0; i < header_.width; ++i) {
        if (!read_pixel(line_buffer[i])) {
            return false;
        }
    }

    return true;
}

void ImageHandler::seek_line(uint16_t line)
{
    if (!file_) return;

    long offset = IMG_TGA_HEAD_SIZE + (static_cast<long>(line) * header_.width * 3);
    fseek(file_, offset, SEEK_SET);
}

HsvColor ImageHandler::rgb_to_hsv(const RgbColor& rgb)
{
    HsvColor hsv;
    uint8_t rgb_min = std::min(std::min(rgb.r, rgb.g), rgb.b);
    uint8_t rgb_max = std::max(std::max(rgb.r, rgb.g), rgb.b);
    
    hsv.v = rgb_max;
    if (hsv.v == 0) {
        hsv.h = hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * static_cast<uint16_t>(rgb_max - rgb_min) / hsv.v;
    if (hsv.s == 0) {
        hsv.h = 0;
        return hsv;
    }

    int16_t h;
    if (rgb_max == rgb.r) {
        h = 0 + 43 * static_cast<int16_t>(rgb.g - rgb.b) / (rgb_max - rgb_min);
    }
    else if (rgb_max == rgb.g) {
        h = 85 + 43 * static_cast<int16_t>(rgb.b - rgb.r) / (rgb_max - rgb_min);
    }
    else {
        h = 171 + 43 * static_cast<int16_t>(rgb.r - rgb.g) / (rgb_max - rgb_min);
    }

    hsv.h = h < 0 ? h + 255 : h;
    return hsv;
}

RgbColor ImageHandler::hsv_to_rgb(const HsvColor& hsv)
{
    RgbColor rgb;
    
    // Grayscale
    if (hsv.s == 0) {
        rgb.r = rgb.g = rgb.b = hsv.v;
        return rgb;
    }

    uint8_t region = hsv.h / 43;
    uint8_t remainder = (hsv.h - (region * 43)) * 6;

    uint8_t p = (hsv.v * (255 - hsv.s)) >> 8;
    uint8_t q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    uint8_t t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

AptColor ImageHandler::rgb_to_apt(const RgbColor& rgb)
{
    AptColor apt;
    // Convert 8-bit RGB components to 4-bit values
    uint8_t r = (rgb.r >> 4) & 0xF;
    uint8_t g = (rgb.g >> 4) & 0xF;
    uint8_t b = (rgb.b >> 4) & 0xF;
    
    // Create lookup table index
    uint16_t val = (r << 8) + (g << 4) + b;
    
    apt.h = (r * 255) / 15;
    apt.sv = (g * 255) / 15;
    
    return apt;
}

RgbColor ImageHandler::apt_to_rgb(const AptColor& apt)
{
    RgbColor rgb;
    
    uint8_t r = (apt.h * 15) / 255;
    uint8_t g = (apt.sv * 15) / 255;
    uint8_t b = g;  // Simplified mapping
    
    rgb.r = (r << 4) | r;  // Expand to 8 bits
    rgb.g = (g << 4) | g;
    rgb.b = (b << 4) | b;
    
    return rgb;
}

} // namespace apt_encoder
} // namespace gr