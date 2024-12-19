// sgp4_orbit.hpp
#ifndef INCLUDED_APT_ENCODER_SGP4_ORBIT_HPP
#define INCLUDED_APT_ENCODER_SGP4_ORBIT_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <libsgp4/SGP4.h>
#include <libsgp4/Tle.h>
#include <libsgp4/Observer.h>
#include <libsgp4/CoordTopocentric.h>
#include <libsgp4/DateTime.h>
#include <libsgp4/Util.h>

namespace gr {
namespace apt_encoder {

class SGP4Orbit {
public:
    struct SatPosition {
        double elevation;   // 仰角（度）
        double azimuth;    // 方位角（度）
        double range;      // 距離（km）
        double velocity;   // 視線速度（km/s）
        double time;       // 時刻（Unix時間）
        bool is_visible;   // 可視性
        double latitude;   // 衛星緯度（度）
        double longitude;  // 衛星経度（度）
        double altitude;   // 衛星高度（km）
    };

    struct GroundStation {
        double latitude;    // 緯度（度）
        double longitude;   // 経度（度）
        double altitude;    // 高度（m）
    };

    // 定数定義
    static constexpr double EARTH_RADIUS = 6378.137;  // 地球半径（km）
    static constexpr double MIN_RANGE = 800.0;        // 最小可視距離（km）
    static constexpr double MAX_RANGE = 3000.0;       // 最大可視距離（km）
    static constexpr double TYPICAL_VELOCITY = 7.8;   // 典型的なLEO衛星速度（km/s）

    const GroundStation& get_ground_station() const;

    SGP4Orbit();
    ~SGP4Orbit() = default;

    void set_tle(const std::string& line1, const std::string& line2);
    void set_orbital_elements(
        double inclination,
        double raan,
        double eccentricity,
        double arg_perigee,
        double mean_anomaly,
        double mean_motion,
        double bstar = 0.0
    );
    void set_ground_station(const GroundStation& station);
    void set_epoch_time(double unix_time);
    double get_epoch_time() const { return datetime_to_unix(epoch_time_); }
    SatPosition calculate_position(double unix_time) const;
    std::vector<SatPosition> predict_passes(
        time_t start_time,
        time_t end_time,
        double time_step = 60.0
    ) const;

private:
    std::unique_ptr<libsgp4::SGP4> sgp4_;
    std::unique_ptr<libsgp4::Observer> observer_;
    GroundStation ground_station_;
    libsgp4::DateTime epoch_time_;

    static inline libsgp4::DateTime unix_to_datetime(double unix_time) {
        double jd = (unix_time / 86400.0) + 2440587.5;
        return libsgp4::DateTime(jd);
    }

    static inline double datetime_to_unix(const libsgp4::DateTime& dt) {
        return (dt.ToJulian() - 2440587.5) * 86400.0;
    }
};

} // namespace apt_encoder
} // namespace gr

#endif // INCLUDED_APT_ENCODER_SGP4_ORBIT_HPP