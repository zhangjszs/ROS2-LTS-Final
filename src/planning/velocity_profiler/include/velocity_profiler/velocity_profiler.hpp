#pragma once

#include <utility>
#include <vector>

#include "velocity_profiler/profiler_params.hpp"

namespace velocity_profiler {

class VelocityProfiler {
   public:
    explicit VelocityProfiler(const ProfilerLimits& limits = ProfilerLimits{});

    void SetLimits(const ProfilerLimits& limits) noexcept { limits_ = limits; }
    [[nodiscard]] const ProfilerLimits& GetLimits() const noexcept { return limits_; }

    /**
     * @brief 为二维几何离散点序列计算时变最优速度剖面 v*(s)
     * @param raw_points 原始二维坐标序列 [(x, y), ...]
     * @param current_speed 车辆当前瞬时车速 (m/s)
     * @return std::vector<ProfilePoint> 优化后的轨迹点详细信息
     */
    [[nodiscard]] std::vector<ProfilePoint> ComputeProfile(const std::vector<std::pair<double, double>>& raw_points,
                                                           double current_speed = 0.0) const;

    static double NormalizeAngle(double angle) noexcept;

   private:
    void ComputeGeometry(std::vector<ProfilePoint>& pts) const;
    void ComputeCorneringLimits(std::vector<ProfilePoint>& pts) const;
    void BackwardPass(std::vector<ProfilePoint>& pts) const;
    void ForwardPass(std::vector<ProfilePoint>& pts, double current_speed) const;

    ProfilerLimits limits_;
};

}  // namespace velocity_profiler
