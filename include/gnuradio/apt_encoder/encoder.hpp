// include/gnuradio/apt_encoder/encoder.hpp
#ifndef INCLUDED_APT_ENCODER_H
#define INCLUDED_APT_ENCODER_H

#include <gnuradio/apt_encoder/api.hpp>
#include <gnuradio/sync_block.h>

namespace gr {
namespace apt_encoder {

/*!
 * \brief APT signal encoder block
 *
 * This block generates APT signals from image files.
 */
class GR_APT_ENCODER_API encoder : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<encoder> sptr;

    /*!
     * \brief Create an APT encoder
     *
     * \param image_file Primary image file path (TGA format)
     * \param second_file Secondary image file path (optional)
     * \param sample_rate Output sample rate
     * \param carrier_frequency Carrier frequency
     * \param loop Enable looping of image data
     * \param mode Channel B data mode (R,G,B,N,Y,C)
     */
    static sptr make(const std::string& image_file,
                    const std::string& second_file = "",
                    double sample_rate = 24000,
                    double carrier_frequency = 2400,
                    bool loop = false,
                    char mode = 'N');

    /*!
     * \brief Estimate output duration in seconds
     *
     * \return Estimated duration of the output signal in seconds
     */
    virtual double estimate_duration() const = 0;

    /*!
     * \brief Estimate total number of output samples
     *
     * \return Estimated number of samples that will be produced
     */
    virtual uint64_t estimate_samples() const = 0;

    /*!
     * \brief Calculate satellite path length in kilometers
     *
     * \return Estimated ground track length based on signal duration
     */
    virtual double calculate_path_length() const = 0;

    /*!
     * \brief Reset the encoder state
     *
     * Resets all internal counters, buffers, and reinitializes image files.
     * This allows the encoder to start fresh when GRC is stopped and restarted.
     */
    virtual void reset() = 0;
};

} // namespace apt_encoder
} // namespace gr

#endif /* INCLUDED_APT_ENCODER_H */