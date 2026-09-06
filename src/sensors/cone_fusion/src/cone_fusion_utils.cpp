#include "cone_fusion_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <sstream>
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
            std::ostringstream oss;
            oss << "dt=" << dt << "s"
                << " distance_jump=" << distance_jump << "m"
                << " allowed_distance_jump=" << allowed_distance_jump << "m"
                << " heading_jump=" << heading_jump << "rad";
            *reason = oss.str();
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

}  // namespace cone_fusion_utils
