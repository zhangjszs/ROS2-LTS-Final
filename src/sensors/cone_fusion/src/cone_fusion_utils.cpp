#include "cone_fusion_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <numbers>
#include <ranges>
#include <span>
#include <string>

namespace cone_fusion_utils {

constexpr double kPi = std::numbers::pi_v<double>;

double NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

bool IsVehicleStateJumpAbnormal(const VehicleState& last_state, const VehicleState& current_state, double dt,
                                double base_jump_threshold, double speed_margin, double min_dt, double max_dt,
                                double heading_threshold, std::string* reason) {
    if (dt <= 0.0 || dt > max_dt) {
        return false;
    }

    const double dx = current_state.x - last_state.x;
    const double dy = current_state.y - last_state.y;
    const double distance_jump = std::hypot(dx, dy);
    const double heading_jump = std::fabs(NormalizeAngle(current_state.theta - last_state.theta));
    const double bounded_dt = std::max(dt, min_dt);
    const double speed = std::max(std::fabs(current_state.v), std::fabs(last_state.v));
    const double allowed_distance_jump = base_jump_threshold + speed * bounded_dt * speed_margin;

    if (distance_jump > allowed_distance_jump || heading_jump > heading_threshold) {
        if (reason) {
            reason->clear();
            std::format_to(std::back_inserter(*reason),
                           "dt={}s distance_jump={}m allowed_distance_jump={}m heading_jump={}rad",
                           dt, distance_jump, allowed_distance_jump, heading_jump);
        }
        return true;
    }

    if (reason) {
        reason->clear();
    }
    return false;
}

uint32_t ConfidenceToPercent(const float* confidence_data, size_t data_size, size_t index) {
    if (index >= data_size) {
        return 0;
    }

    double confidence = static_cast<double>(confidence_data[index]);
    if (!std::isfinite(confidence)) {
        return 0;
    }

    if (confidence <= 1.0) {
        confidence *= 100.0;
    }
    if (confidence < 0.0) {
        confidence = 0.0;
    } else if (confidence > 100.0) {
        confidence = 100.0;
    }

    return static_cast<uint32_t>(std::lround(confidence));
}

bool IsPointFinite(const RawPoint& pt) {
    return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}

bool IsDistanceValid(const RawPoint& pt, double min_dist, double max_dist) {
    const double d2 = pt.x * pt.x + pt.y * pt.y;
    return d2 >= min_dist * min_dist && d2 <= max_dist * max_dist;
}

bool IsFieldOfViewValid(const RawPoint& pt, double min_fov, double max_fov) {
    const double angle = std::atan2(pt.y, pt.x);
    return angle >= min_fov && angle <= max_fov;
}

bool IsConfidenceValid(uint32_t confidence, uint32_t min_conf) {
    return confidence >= min_conf;
}

std::vector<size_t> FilterConesPipeline(std::span<const RawPoint> points,
                                        std::span<const uint32_t> confidences,
                                        const ConeCleaningParams& params) {
    if (points.empty()) {
        return {};
    }

    const size_t n = points.size();

    // C++20 惰性流式管道：多重清洗过滤组合为单一视图管道，无任何中间 vector 分配
    auto indices = std::views::iota(size_t{0}, n)
        | std::views::filter([&](size_t i) {
            return IsPointFinite(points[i]);
        })
        | std::views::filter([&](size_t i) {
            return IsDistanceValid(points[i], params.min_distance, params.max_distance);
        })
        | std::views::filter([&](size_t i) {
            return IsFieldOfViewValid(points[i], params.min_fov_rad, params.max_fov_rad);
        })
        | std::views::filter([&](size_t i) {
            const uint32_t conf = (i < confidences.size()) ? confidences[i] : 100u;
            return IsConfidenceValid(conf, params.min_confidence);
        });

    std::vector<size_t> valid_indices;
    valid_indices.reserve(n);
    for (size_t i : indices) {
        valid_indices.push_back(i);
    }
    return valid_indices;
}

}  // namespace cone_fusion_utils
