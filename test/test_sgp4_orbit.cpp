#include <gtest/gtest.h>
#include "sgp4_orbit.hpp"
#include <cmath>
#include <chrono>
#include <iostream>
#include <iomanip>

class SGP4OrbitTest : public ::testing::Test {
protected:
    void SetUp() override {
        // NOAA-19の有効なTLE（時刻を現在に近い値に更新）
        line1_ = "1 33591U 09005A   24018.50000000  .00000045  00000-0  51014-4 0  9990";
        line2_ = "2 33591  98.7904 312.5878 0013594 111.9829 248.2634 14.12501932766033";
        
        // 東新潟自動車学校の位置
        station_.latitude = 37.9212;
        station_.longitude = 139.0813;
        station_.altitude = 2.0;

        // Create orbit instance
        orbit_ = std::make_unique<gr::apt_encoder::SGP4Orbit>();
    }

    void print_satellite_position(const gr::apt_encoder::SGP4Orbit::SatPosition& pos) {
        time_t time_val = static_cast<time_t>(pos.time);
        std::tm* tm_info = std::gmtime(&time_val);
        
        std::cout << "\nSatellite Position:" << std::endl
                  << "  Time: " << std::put_time(tm_info, "%Y-%m-%d %H:%M:%S UTC") << std::endl
                  << "  Azimuth: " << pos.azimuth << "°" << std::endl
                  << "  Elevation: " << pos.elevation << "°" << std::endl
                  << "  Range: " << pos.range << " km" << std::endl
                  << "  Velocity: " << pos.velocity << " km/s" << std::endl
                  << "  Visible: " << (pos.is_visible ? "Yes" : "No") << std::endl;
    }

    std::string line1_;
    std::string line2_;
    gr::apt_encoder::SGP4Orbit::GroundStation station_;
    std::unique_ptr<gr::apt_encoder::SGP4Orbit> orbit_;
};

TEST_F(SGP4OrbitTest, TLEValidation) {
    libsgp4::Tle tle("NOAA 19", line1_, line2_);
    
    // NOAAの軌道パラメータの検証
    EXPECT_GT(tle.MeanMotion(), 14.0) << "Mean motion too low for NOAA";
    EXPECT_LT(tle.MeanMotion(), 14.5) << "Mean motion too high for NOAA";
    EXPECT_GT(tle.Inclination(true), 98.0) << "Inclination too low for sun-synchronous orbit";
    EXPECT_LT(tle.Inclination(true), 99.0) << "Inclination too high for sun-synchronous orbit";
    EXPECT_LT(tle.Eccentricity(), 0.1) << "Eccentricity too high for LEO";
    
    // 軌道周期の検証
    double period = 1440.0 / tle.MeanMotion();  // 分単位
    EXPECT_GT(period, 90.0) << "Orbital period too short for LEO";
    EXPECT_LT(period, 120.0) << "Orbital period too long for LEO";
}

TEST_F(SGP4OrbitTest, BasicPositionCalculation) {
    ASSERT_NO_THROW(orbit_->set_tle(line1_, line2_));
    ASSERT_NO_THROW(orbit_->set_ground_station(station_));

    auto epoch_time = orbit_->get_epoch_time();
    std::cout << "Calculating position at epoch time: " << epoch_time << std::endl;
    
    auto pos = orbit_->calculate_position(epoch_time);
    print_satellite_position(pos);

    // 基本的な妥当性チェック
    constexpr double MIN_RANGE = 800.0;        // 最小可視距離（km）
    constexpr double MAX_RANGE = 3000.0;       // 最大可視距離（km）

    EXPECT_GE(pos.range, MIN_RANGE) << "Range too low for LEO satellite";
    EXPECT_LE(pos.range, MAX_RANGE) << "Range too high for LEO satellite";
    
    EXPECT_GE(pos.elevation, -90.0) << "Invalid elevation angle";
    EXPECT_LE(pos.elevation, 90.0) << "Invalid elevation angle";
    
    EXPECT_GE(pos.azimuth, 0.0) << "Invalid azimuth angle";
    EXPECT_LE(pos.azimuth, 360.0) << "Invalid azimuth angle";
    
    // 典型的なLEO衛星の速度範囲をチェック
    EXPECT_LE(std::abs(pos.velocity), 8.0) << "Unrealistic velocity for LEO satellite";
}

TEST_F(SGP4OrbitTest, PassPrediction) {
    ASSERT_NO_THROW(orbit_->set_tle(line1_, line2_));
    ASSERT_NO_THROW(orbit_->set_ground_station(station_));

    // エポックから30分のパスを予測（短い時間で試験）
    double epoch_time = orbit_->get_epoch_time();
    double end_time = epoch_time + 1800.0;  // 30分後

    std::cout << "Predicting passes from " << epoch_time 
              << " to " << end_time << std::endl;

    auto positions = orbit_->predict_passes(epoch_time, end_time, 30.0);
    ASSERT_FALSE(positions.empty()) << "No positions calculated";

    // パスの統計を収集
    double max_elevation = -90.0;
    int visible_count = 0;
    double prev_time = 0;

    for (const auto& pos : positions) {
        if (pos.elevation > max_elevation) {
            max_elevation = pos.elevation;
            print_satellite_position(pos);  // 最大仰角時の位置を表示
        }
        if (pos.elevation > 0) {
            visible_count++;
        }
        
        // 時間が単調増加することを確認
        if (prev_time != 0) {
            EXPECT_GT(pos.time, prev_time) << "Positions not in chronological order";
        }
        prev_time = pos.time;
    }

    std::cout << "\nPass prediction summary:" << std::endl
              << "  Total positions: " << positions.size() << std::endl
              << "  Visible positions: " << visible_count << std::endl
              << "  Maximum elevation: " << max_elevation << "°" << std::endl;

    // 結果の検証
    EXPECT_GT(positions.size(), 0) << "No positions calculated";
    EXPECT_GE(max_elevation, -90.0) << "Invalid maximum elevation";
    EXPECT_LE(max_elevation, 90.0) << "Invalid maximum elevation";
}

TEST_F(SGP4OrbitTest, OrbitParameterValidation) {
    // 無効なパラメータのテスト
    EXPECT_THROW(orbit_->set_orbital_elements(
        -1.0,    // 無効な傾斜角
        180.0, 0.001, 0.0, 0.0, 14.1),
        std::runtime_error) << "Should reject negative inclination";
    
    EXPECT_THROW(orbit_->set_orbital_elements(
        98.7, 180.0, 1.1,    // 無効な離心率
        0.0, 0.0, 14.1),
        std::runtime_error) << "Should reject eccentricity >= 1";
    
    EXPECT_THROW(orbit_->set_orbital_elements(
        98.7, 180.0, 0.001, 0.0, 0.0, 0.0),  // 無効な平均運動
        std::runtime_error) << "Should reject zero mean motion";

    // 有効な値での地上局設定のテスト
    gr::apt_encoder::SGP4Orbit::GroundStation valid_station;
    valid_station.latitude = 35.0;
    valid_station.longitude = 135.0;
    valid_station.altitude = 100.0;
    
    EXPECT_NO_THROW(orbit_->set_ground_station(valid_station))
        << "Should accept valid ground station parameters";
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}