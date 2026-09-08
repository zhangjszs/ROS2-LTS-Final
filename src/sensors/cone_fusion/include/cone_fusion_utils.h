#pragma once

#include <cmath>
#include <compare>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace cone_fusion_utils {

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;

    auto operator<=>(const VehicleState&) const = default;
};

struct RawPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    auto operator<=>(const RawPoint&) const = default;
};

struct ConeCleaningParams {
    double min_distance = 0.5;
    double max_distance = 30.0;
    double min_fov_rad = -2.0;
    double max_fov_rad = 2.0;
    uint32_t min_confidence = 10;

    auto operator<=>(const ConeCleaningParams&) const = default;
};

/**
 * @brief 融合感知候选锥桶结构体（ConeCandidate）
 * 包含 3D 几何坐标、置信度以及颜色分类标签，具备完整的 C++20 三路比较器支持。
 */
struct ConeCandidate {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double confidence = 0.0;
    uint32_t color = 0;

    auto operator<=>(const ConeCandidate&) const = default;
};

double NormalizeAngle(double angle);

bool IsVehicleStateJumpAbnormal(const VehicleState& last_state, const VehicleState& current_state, double dt,
                                double base_jump_threshold, double speed_margin, double min_dt, double max_dt,
                                double heading_threshold, std::string* reason = nullptr);

uint32_t ConfidenceToPercent(const float* confidence_data, size_t data_size, size_t index);

// 单点清洗断言函数（供流式管道组合）
bool IsPointFinite(const RawPoint& pt);
bool IsDistanceValid(const RawPoint& pt, double min_dist, double max_dist);
bool IsFieldOfViewValid(const RawPoint& pt, double min_fov, double max_fov);
bool IsConfidenceValid(uint32_t confidence, uint32_t min_conf);

// C++20 std::views::filter 惰性流式清洗流水线：按需组合多重过滤阶段，零临时 vector 堆内存分配
std::vector<size_t> FilterConesPipeline(std::span<const RawPoint> points, std::span<const uint32_t> confidences,
                                        const ConeCleaningParams& params);

}  // namespace cone_fusion_utils
