#pragma once

#include <cone_types.h>

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <vector>

#include "vehicle_simulator/bicycle_model.hpp"

namespace simulation {

/**
 * @brief 赛道锥桶定义（#54：从 SensorSimulator 抽到纯 std core，供无 ROS 单测复用）
 */
struct TrackCone {
    double x{0.0};
    double y{0.0};
    uint32_t type{huat_cone::BLUE};  // huat_cone::Color
    uint32_t id{0};
};

/**
 * @brief 中性探测结果（不含 ROS 消息类型；由适配层转换为 HuatCone）
 */
struct DetectedCone {
    uint32_t id{0};
    uint32_t type{0};
    uint32_t confidence{0};
    float x_base{0.0f};
    float y_base{0.0f};
    float x_global{0.0f};
    float y_global{0.0f};
};

/**
 * @brief 感知几何核（纯 std，header-only）：距离粗筛 + base_link 坐标变换 + 水平 FOV 判定
 *        + 可选测距高斯噪声。
 *
 * 语义与 #54 重构前的 `SensorSimulator::GeneratePerceivedCones` 逐位一致（由既有
 * `SensorSimulatorTest` + 本核单测共同锁定）：
 *  - 距离粗筛：`dx^2 + dy^2` 必须落在 `[0.25, max_range^2]`；
 *  - base_link 变换：`x_base = dx*cosθ + dy*sinθ`，`y_base = -dx*sinθ + dy*cosθ`；
 *  - 车体前方判定：`x_base > 0.2`；
 *  - 水平 FOV：`|atan2(y_base, x_base)| <= fov_deg/2`；
 *  - `noise_stddev > 0` 时，每个命中锥桶按 x→y 顺序各抽一次 `N(0, noise_stddev)`，
 *    rng 消耗顺序固定（保证同种子可复现）；
 *  - 命中项 `confidence = 95`。
 *
 * @param cones 全局锥桶列表
 * @param state 车辆当前状态
 * @param fov_deg 水平视场角（度）
 * @param max_range 最大探测距离（米）
 * @param noise_stddev 测距高斯噪声标准差（米）；<=0 表示不采样
 * @param rng 随机数引擎（由调用方持有，保证可复现；noise<=0 时不消耗）
 */
[[nodiscard]] inline std::vector<DetectedCone> PredictVisibleCones(const std::vector<TrackCone>& cones,
                                                                   const VehicleState& state, double fov_deg,
                                                                   double max_range, double noise_stddev,
                                                                   std::mt19937& rng) {
    std::vector<DetectedCone> out;
    constexpr double kPi = std::numbers::pi_v<double>;
    constexpr double kDegToRad = kPi / 180.0;

    const double half_fov_rad = (fov_deg * 0.5) * kDegToRad;
    const double max_range_sq = max_range * max_range;
    std::normal_distribution<double> dist(0.0, (noise_stddev > 0.0) ? noise_stddev : 1.0);

    const double cos_th = std::cos(state.theta);
    const double sin_th = std::sin(state.theta);

    for (const auto& cone : cones) {
        // 1. 全局坐标平移到车辆质心
        const double dx = cone.x - state.x;
        const double dy = cone.y - state.y;
        const double dist_sq = dx * dx + dy * dy;

        // 距离粗筛
        if (dist_sq > max_range_sq || dist_sq < 0.25)
            continue;

        // 2. 旋转到车体坐标系 (base_link: X 朝前, Y 朝左)
        const double x_base = dx * cos_th + dy * sin_th;
        const double y_base = -dx * sin_th + dy * cos_th;

        // 必须在车体前方
        if (x_base <= 0.2)
            continue;

        // 3. 水平视场角 (FOV) 判定
        const double angle = std::atan2(y_base, x_base);
        if (std::abs(angle) > half_fov_rad)
            continue;

        // 4. 生成探测结果并加入测距高斯噪声
        const double n_x = (noise_stddev > 0.0) ? dist(rng) : 0.0;
        const double n_y = (noise_stddev > 0.0) ? dist(rng) : 0.0;

        DetectedCone detected;
        detected.id = cone.id;
        detected.type = cone.type;
        detected.confidence = 95u;
        detected.x_base = static_cast<float>(x_base + n_x);
        detected.y_base = static_cast<float>(y_base + n_y);
        detected.x_global = static_cast<float>(cone.x + n_x);
        detected.y_global = static_cast<float>(cone.y + n_y);
        out.push_back(detected);
    }

    return out;
}

}  // namespace simulation
