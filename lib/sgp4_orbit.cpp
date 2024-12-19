#include "sgp4_orbit.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace gr {
namespace apt_encoder {

constexpr double SGP4Orbit::EARTH_RADIUS;
constexpr double SGP4Orbit::MIN_RANGE;
constexpr double SGP4Orbit::MAX_RANGE;
constexpr double SGP4Orbit::TYPICAL_VELOCITY;


SGP4Orbit::SGP4Orbit()
    : epoch_time_(libsgp4::DateTime::Now(true))
{
    // デフォルトの地上局設定
    ground_station_.latitude = 35.0;
    ground_station_.longitude = 135.0;
    ground_station_.altitude = 0.0;

    observer_ = std::make_unique<libsgp4::Observer>(
        ground_station_.latitude * M_PI / 180.0,
        ground_station_.longitude * M_PI / 180.0,
        ground_station_.altitude / 1000.0
    );

    // デフォルトのNOAA-19 TLE
    try {
        const char* line1 = "1 33591U 09005A   24018.50000000  .00000045  00000-0  51014-4 0  9990";
        const char* line2 = "2 33591  98.7904 312.5878 0013594 111.9829 248.2634 14.12501932766033";
        set_tle(line1, line2);
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to set default TLE: " << e.what() << std::endl;
        throw;
    }
}

const SGP4Orbit::GroundStation& SGP4Orbit::get_ground_station() const {
    return ground_station_;
}

void SGP4Orbit::set_tle(const std::string& line1, const std::string& line2)
{
    try {
        // TLEを直接構築
        libsgp4::Tle tle("NOAA 19", line1, line2);
        sgp4_ = std::make_unique<libsgp4::SGP4>(tle);
        epoch_time_ = tle.Epoch();

        // TLE情報をデバッグ出力
        std::cerr << "TLE set successfully:" << std::endl
                  << "Satellite: NOAA 19" << std::endl
                  << "Epoch: " << epoch_time_ << std::endl
                  << "Inclination: " << tle.Inclination(true) << "°" << std::endl
                  << "Period: " << (1440.0 / tle.MeanMotion()) << " minutes" << std::endl;
    }
    catch (const std::exception& e) {
        throw std::runtime_error("Failed to initialize SGP4 with TLE: " + std::string(e.what()));
    }
}

void SGP4Orbit::set_orbital_elements(
    double inclination,
    double raan,
    double eccentricity,
    double arg_perigee,
    double mean_anomaly,
    double mean_motion,
    double bstar)
{
    // パラメータの検証
    if (inclination < 0.0 || inclination > 180.0) {
        throw std::runtime_error("Inclination must be between 0 and 180 degrees");
    }
    if (eccentricity < 0.0 || eccentricity >= 1.0) {
        throw std::runtime_error("Eccentricity must be between 0 and < 1.0");
    }
    if (mean_motion <= 0.0 || mean_motion > 17.0) {
        throw std::runtime_error("Mean motion must be positive and < 17 orbits/day");
    }

    try {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto* utc = std::gmtime(&time);
        
        std::ostringstream line1, line2;
        
        // Line 1 - 正確な69文字の長さに調整
        line1 << "1 33591U 09005A   "
              << std::setfill('0') 
              << std::setw(2) << (utc->tm_year % 100)
              << std::setw(3) << (utc->tm_yday + 1)
              << ".00000000  "
              << std::setprecision(8) << std::fixed 
              << " .00000000  00000-0  "
              << std::setw(8) << bstar
              << " 0  9999";

        // Line 2 - 正確な69文字の長さに調整
        line2 << "2 33591  "
              << std::fixed 
              << std::setw(8) << std::setprecision(4) << inclination
              << " " << std::setw(8) << std::setprecision(4) << raan
              << " " << std::setw(7) << std::setprecision(7) << eccentricity
              << " " << std::setw(8) << std::setprecision(4) << arg_perigee
              << " " << std::setw(8) << std::setprecision(4) << mean_anomaly
              << " " << std::setw(11) << std::setprecision(8) << mean_motion
              << "00000";

        // 厳密な長さチェック
        std::string l1 = line1.str();
        std::string l2 = line2.str();
        
        if (l1.length() != 69 || l2.length() != 69) {
            throw std::runtime_error("Invalid TLE string length");
        }

        set_tle(l1, l2);
    }
    catch (const std::exception& e) {
        throw std::runtime_error("Failed to create orbit from elements: " + std::string(e.what()));
    }
}

void SGP4Orbit::set_ground_station(const GroundStation& station)
{
    // 座標の検証
    if (station.latitude < -90.0 || station.latitude > 90.0) {
        throw std::runtime_error("Latitude must be between -90 and 90 degrees");
    }
    if (station.longitude < -180.0 || station.longitude > 180.0) {
        throw std::runtime_error("Longitude must be between -180 and 180 degrees");
    }
    if (station.altitude < -500.0 || station.altitude > 9000.0) {
        throw std::runtime_error("Altitude must be between -500 and 9000 meters");
    }

    ground_station_ = station;
    observer_ = std::make_unique<libsgp4::Observer>(
        station.latitude * M_PI / 180.0,
        station.longitude * M_PI / 180.0,
        station.altitude / 1000.0  // メートルからキロメートルに変換
    );
}

SGP4Orbit::SatPosition SGP4Orbit::calculate_position(double unix_time) const
{
    static double last_debug_time = 0;
    bool should_debug = (unix_time - last_debug_time) >= 1.0;

    try {
        if (should_debug) {
            std::cerr << "\nCalculating satellite position:"
                      << "\n  Unix time: " << unix_time
                      << "\n  Epoch time: " << datetime_to_unix(epoch_time_) << std::endl;
        }

        // エポックからの経過時間（分）を計算
        double minutes_since_epoch = (unix_time - datetime_to_unix(epoch_time_)) / 60.0;

        if (should_debug) {
            std::cerr << "  Minutes since epoch: " << minutes_since_epoch << std::endl;
        }

        // SGP4で位置を計算
        libsgp4::Eci eci = sgp4_->FindPosition(minutes_since_epoch);
        libsgp4::CoordTopocentric topo = observer_->GetLookAngle(eci);

        // 結果を格納
        SatPosition result;
        result.azimuth = libsgp4::Util::RadiansToDegrees(topo.azimuth);
        result.elevation = libsgp4::Util::RadiansToDegrees(topo.elevation);
        result.range = topo.range;
        result.velocity = topo.range_rate;
        result.time = unix_time;
        result.is_visible = (result.elevation > 0.0);

        // ECI座標から地理座標に変換
        libsgp4::CoordGeodetic geo = eci.ToGeodetic();
        result.latitude = libsgp4::Util::RadiansToDegrees(geo.latitude);
        result.longitude = libsgp4::Util::RadiansToDegrees(geo.longitude);
        result.altitude = geo.altitude;

        if (should_debug) {
            std::cerr << "Calculated position:"
                      << "\n  Lat: " << result.latitude << "°"
                      << "\n  Lon: " << result.longitude << "°"
                      << "\n  Alt: " << result.altitude << " km"
                      << "\n  Az: " << result.azimuth << "°"
                      << "\n  El: " << result.elevation << "°"
                      << "\n  Range: " << result.range << " km"
                      << "\n  Velocity: " << result.velocity << " km/s"
                      << std::endl;
            last_debug_time = unix_time;
        }

        return result;
    }
    catch (const std::exception& e) {
        std::cerr << "Error in calculate_position: " << e.what() << std::endl;
        throw;
    }
}

std::vector<SGP4Orbit::SatPosition> SGP4Orbit::predict_passes(
    time_t start_time,
    time_t end_time,
    double time_step) const
{
    std::vector<SatPosition> positions;

    if (start_time >= end_time) {
        throw std::runtime_error("End time must be after start time");
    }
    if (time_step <= 0.0) {
        throw std::runtime_error("Time step must be positive");
    }

    for (double t = start_time; t <= end_time; t += time_step) {
        try {
            auto pos = calculate_position(t);
            positions.push_back(pos);

            // 衛星が可視の場合、より細かい時間間隔でサンプリング
            if (pos.elevation > 0.0) {
                double fine_step = time_step / 5.0;
                for (double ft = t - time_step + fine_step; ft < t; ft += fine_step) {
                    if (ft >= start_time) {
                        positions.push_back(calculate_position(ft));
                    }
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error calculating position at time " << t 
                     << ": " << e.what() << std::endl;
            continue;
        }
    }

    // 時間順にソート
    std::sort(positions.begin(), positions.end(),
              [](const SatPosition& a, const SatPosition& b) {
                  return a.time < b.time;
              });

    return positions;
}

} // namespace apt_encoder
} // namespace gr