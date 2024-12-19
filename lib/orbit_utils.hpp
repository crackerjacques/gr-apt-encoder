// orbit_utils.hpp

#ifndef INCLUDED_APT_ENCODER_ORBIT_UTILS_HPP
#define INCLUDED_APT_ENCODER_ORBIT_UTILS_HPP

#include "sgp4_orbit.hpp"
#include <vector>

namespace gr {
namespace apt_encoder {

/**
 * 衛星の可視性を判定
 * @param pos 衛星位置情報
 * @param min_elevation 最低仰角（度）
 * @return 可視であればtrue
 */
bool is_satellite_visible(const SGP4Orbit::SatPosition& pos, double min_elevation = 0.0);

/**
 * パスを予測し、詳細な位置情報を生成
 * @param orbit 軌道オブジェクト
 * @param start_time 開始時刻（Unix時間）
 * @param end_time 終了時刻（Unix時間）
 * @param time_step サンプリング間隔（秒）
 * @return 予測された位置情報のリスト
 */
std::vector<SGP4Orbit::SatPosition> predict_passes(
    const SGP4Orbit& orbit,
    time_t start_time,
    time_t end_time,
    double time_step = 60.0
);

/**
 * パス中の最大仰角を計算
 * @param positions 位置情報のリスト
 * @return 最大仰角（度）
 */
double calculate_max_elevation(const std::vector<SGP4Orbit::SatPosition>& positions);

/**
 * パスの継続時間を計算
 * @param positions 位置情報のリスト
 * @param time_step サンプリング間隔（秒）
 * @return パス継続時間（秒）
 */
double calculate_pass_duration(const std::vector<SGP4Orbit::SatPosition>& positions,
                             double time_step);

} // namespace apt_encoder
} // namespace gr

#endif // INCLUDED_APT_ENCODER_ORBIT_UTILS_HPP