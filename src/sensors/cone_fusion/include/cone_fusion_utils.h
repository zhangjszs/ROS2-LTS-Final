#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace cone_fusion_utils {

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;
};

double NormalizeAngle(double angle);

bool IsVehicleStateJumpAbnormal(const VehicleState& last_state, const VehicleState& current_state, double dt,
                                double base_jump_threshold, double speed_margin, double min_dt, double max_dt,
                                double heading_threshold, std::string* reason = nullptr);

uint32_t ConfidenceToPercent(const float* confidence_data, size_t data_size, size_t index);

}  // namespace cone_fusion_utils
