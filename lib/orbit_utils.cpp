// orbit_utils.cpp
#include "orbit_utils.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <iostream>

namespace gr {
namespace apt_encoder {

bool is_satellite_visible(const SGP4Orbit::SatPosition& pos, double min_elevation)
{
    // 地平線より上にあり、最小仰角条件を満たすかチェック
    if (pos.elevation < min_elevation) {
        return false;
    }

    // 有効な距離範囲内かチェック（NOAAの典型的な可視範囲を考慮）
    const double MIN_RANGE = 800.0;  // km
    const double MAX_RANGE = 3000.0; // km
    if (pos.range < MIN_RANGE || pos.range > MAX_RANGE) {
        return false;
    }

    // 速度が妥当な範囲内かチェック
    const double TYPICAL_VELOCITY = 7.8; // km/s
    const double VELOCITY_TOLERANCE = 0.5; // km/s
    if (std::abs(std::abs(pos.velocity) - TYPICAL_VELOCITY) > VELOCITY_TOLERANCE) {
        return false;
    }

    return true;
}

// 地平線との交点を見つける二分探索関数
std::pair<double, double> find_crossing_point(
    const SGP4Orbit& orbit,
    double time1,
    double time2,
    bool finding_aos)
{
    const int MAX_ITERATIONS = 16;
    double t1 = time1;
    double t2 = time2;
    double last_elevation = 0.0;
    
    for (int i = 0; i < MAX_ITERATIONS; i++) {
        double mid_time = (t1 + t2) / 2.0;
        auto pos = orbit.calculate_position(mid_time);
        last_elevation = pos.elevation;

        if (pos.elevation > 0.0) {
            if (finding_aos) {
                t2 = mid_time;
            } else {
                t1 = mid_time;
            }
        } else {
            if (finding_aos) {
                t1 = mid_time;
            } else {
                t2 = mid_time;
            }
        }
        
        if (std::abs(t2 - t1) < 1.0/86400.0) { // 1秒未満の精度
            return std::make_pair(mid_time, last_elevation);
        }
    }
    
    return std::make_pair(time1, last_elevation); // 交点が見つからない場合
}

// パス中の最大仰角を見つける
double find_max_elevation(
    const SGP4Orbit& orbit,
    double start_time,
    double end_time)
{
    const int NUM_POINTS = 20;
    double max_elevation = -90.0;
    double step = (end_time - start_time) / NUM_POINTS;
    
    // まず粗い探索
    double max_time = start_time;
    for (double t = start_time; t <= end_time; t += step) {
        auto pos = orbit.calculate_position(t);
        if (pos.elevation > max_elevation) {
            max_elevation = pos.elevation;
            max_time = t;
        }
    }
    
    // 最大値付近をより細かく探索
    double fine_step = step / 10.0;
    for (double t = max_time - step; t <= max_time + step; t += fine_step) {
        if (t >= start_time && t <= end_time) {
            auto pos = orbit.calculate_position(t);
            max_elevation = std::max(max_elevation, pos.elevation);
        }
    }
    
    return max_elevation;
}

std::vector<SGP4Orbit::SatPosition> predict_passes(
    const SGP4Orbit& orbit,
    time_t start_time,
    time_t end_time,
    double time_step)
{
    std::vector<SGP4Orbit::SatPosition> positions;
    
    // 時間を日単位に変換
    double start_days = static_cast<double>(start_time) / 86400.0;
    double end_days = static_cast<double>(end_time) / 86400.0;
    double step_days = time_step / 86400.0;

    for (double t = start_days; t <= end_days; t += step_days) {
        try {
            auto pos = orbit.calculate_position(t);
            positions.push_back(pos);

            if (pos.elevation > 0.0) {
                // 可視パスが見つかった場合、より細かいステップで計算
                double fine_step = step_days / 10.0;
                for (double ft = t - step_days; ft < t; ft += fine_step) {
                    if (ft >= start_days) {
                        auto fine_pos = orbit.calculate_position(ft);
                        positions.push_back(fine_pos);
                    }
                }
            }
        } catch (const std::exception& e) {
            if (getenv("DEBUG_PASS")) {
                std::cerr << "Error at time " << t << ": " << e.what() << std::endl;
            }
        }
    }

    return positions;
}

double calculate_max_elevation(const std::vector<SGP4Orbit::SatPosition>& positions)
{
    if (positions.empty()) {
        return 0.0;
    }

    return std::max_element(
        positions.begin(),
        positions.end(),
        [](const SGP4Orbit::SatPosition& a, const SGP4Orbit::SatPosition& b) {
            return a.elevation < b.elevation;
        }
    )->elevation;
}

double calculate_pass_duration(const std::vector<SGP4Orbit::SatPosition>& positions,
                             double time_step)
{
    if (positions.empty()) {
        return 0.0;
    }

    size_t visible_count = std::count_if(
        positions.begin(),
        positions.end(),
        [](const SGP4Orbit::SatPosition& pos) {
            return pos.is_visible;
        });

    return visible_count * time_step;
}

} // namespace apt_encoder
} // namespace gr
