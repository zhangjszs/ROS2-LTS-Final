#include "velocity_profiler/velocity_profiler.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>

namespace velocity_profiler {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEps = 1e-6;

}  // namespace

VelocityProfiler::VelocityProfiler(const ProfilerLimits& limits) : limits_(limits) {}

double VelocityProfiler::NormalizeAngle(double angle) noexcept {
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

std::vector<ProfilePoint> VelocityProfiler::ComputeProfile(const std::vector<std::pair<double, double>>& raw_points,
                                                           double current_speed) const {
    if (raw_points.size() < 2) {
        // 几何退化为 0/1 点时无法估计弧长/曲率，不存在“可行驶剖面”。
        // 旧行为给 min_velocity（一个可行驶速度）属 ROS1 #13/#9 缺陷类：“最低速度覆盖横向约束”/
        // “畸形路径仍发可行驶速度”。按 #24 验收矩阵：冲突时必须“约束优先或给出明确不可行状态”，
        // 这里用 0.0（#11 语义下的合法“停车目标”）而非臆造巡航速度。
        std::vector<ProfilePoint> result;
        for (const auto& [x, y] : raw_points) {
            ProfilePoint pt;
            pt.x = x;
            pt.y = y;
            pt.target_speed = 0.0;
            result.push_back(pt);
        }
        return result;
    }

    std::vector<ProfilePoint> pts;
    pts.reserve(raw_points.size());
    for (const auto& [x, y] : raw_points) {
        ProfilePoint pt;
        pt.x = x;
        pt.y = y;
        pts.push_back(pt);
    }

    // 1. 几何特征解算 (弧长、航向与平滑曲率)
    ComputeGeometry(pts);

    // 2. 弯道极限车速规划 (基于向心加速度约束 a_y <= a_y_max)
    ComputeCorneringLimits(pts);

    // 3. 后向制动规划 (保证入弯前有充分距离安全减速)
    BackwardPass(pts);

    // 4. 前向牵引规划 (结合初始车速与电机驱动力限制加速)
    ForwardPass(pts, current_speed);

    // 5. 整合最优车速
    for (auto& pt : pts) {
        pt.target_speed = std::clamp(pt.v_fwd, 0.0, limits_.max_velocity);
    }

    return pts;
}

void VelocityProfiler::ComputeGeometry(std::vector<ProfilePoint>& pts) const {
    const size_t n = pts.size();
    if (n < 2) {
        return;
    }

    // 1. 累计弧长与切线航向
    pts[0].s = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double dx = pts[i].x - pts[i - 1].x;
        double dy = pts[i].y - pts[i - 1].y;
        double ds = std::hypot(dx, dy);
        pts[i].s = pts[i - 1].s + ds;
    }

    for (size_t i = 0; i + 1 < n; ++i) {
        double dx = pts[i + 1].x - pts[i].x;
        double dy = pts[i + 1].y - pts[i].y;
        pts[i].theta = std::atan2(dy, dx);
    }
    pts[n - 1].theta = pts[n - 2].theta;

    // 2. 曲率估计: kappa = dtheta / ds
    std::vector<double> raw_curvature(n, 0.0);
    for (size_t i = 1; i + 1 < n; ++i) {
        double dtheta = NormalizeAngle(pts[i].theta - pts[i - 1].theta);
        double ds = 0.5 * (pts[i + 1].s - pts[i - 1].s);
        if (ds > kEps) {
            raw_curvature[i] = dtheta / ds;
        } else {
            raw_curvature[i] = 0.0;
        }
    }
    raw_curvature[0] = raw_curvature[1];
    raw_curvature[n - 1] = raw_curvature[n - 2];

    // 3. 滑动平均平滑滤波
    const int half_w = std::max(1, limits_.curvature_smoothing_window / 2);
    for (size_t i = 0; i < n; ++i) {
        int left = std::max(0, static_cast<int>(i) - half_w);
        int right = std::min(static_cast<int>(n) - 1, static_cast<int>(i) + half_w);
        double sum_k = 0.0;
        int count = 0;
        for (int j = left; j <= right; ++j) {
            sum_k += raw_curvature[j];
            ++count;
        }
        pts[i].curvature = (count > 0) ? (sum_k / count) : raw_curvature[i];
    }
}

void VelocityProfiler::ComputeCorneringLimits(std::vector<ProfilePoint>& pts) const {
    const double a_y_max = std::max(0.1, limits_.max_lat_accel);

    for (auto& pt : pts) {
        const double abs_k = std::abs(pt.curvature);
        if (abs_k < 1e-4) {
            // 直线或近直线区间，允许最高极速
            pt.v_corner = limits_.max_velocity;
        } else {
            // a_y = v^2 * kappa <= a_y_max ==> v <= sqrt(a_y_max / kappa)
            double v_lim = std::sqrt(a_y_max / abs_k);
            pt.v_corner = std::min(v_lim, limits_.max_velocity);
        }
    }
}

void VelocityProfiler::BackwardPass(std::vector<ProfilePoint>& pts) const {
    const size_t n = pts.size();
    if (n == 0) {
        return;
    }

    // 终点初始条件
    pts[n - 1].v_back = pts[n - 1].v_corner;

    for (size_t i = n - 1; i > 0; --i) {
        const size_t curr = i - 1;
        const size_t next = i;
        const double ds = pts[next].s - pts[curr].s;

        double decel_avail = limits_.max_lon_decel;

        // 摩擦椭圆耦合: (a_x / a_x_max)^2 + (a_y / a_y_max)^2 <= 1
        if (limits_.enable_friction_circle) {
            double v_next = pts[next].v_back;
            double a_y = v_next * v_next * std::abs(pts[next].curvature);
            double ratio = a_y / limits_.max_lat_accel;
            if (ratio < 1.0) {
                decel_avail = limits_.max_lon_decel * std::sqrt(1.0 - ratio * ratio);
            } else {
                decel_avail = 0.2 * limits_.max_lon_decel;  // 保留最小制动力防止数值死锁
            }
        }

        // v_curr^2 <= v_next^2 + 2 * a_decel * ds
        double max_reachable_v = std::sqrt(pts[next].v_back * pts[next].v_back + 2.0 * decel_avail * ds);
        pts[curr].v_back = std::min(pts[curr].v_corner, max_reachable_v);
    }
}

void VelocityProfiler::ForwardPass(std::vector<ProfilePoint>& pts, double current_speed) const {
    const size_t n = pts.size();
    if (n == 0) {
        return;
    }

    // 起点初始条件：结合当前车速与后向规划约束
    double initial_v = (current_speed > limits_.min_velocity) ? current_speed : limits_.min_velocity;
    pts[0].v_fwd = std::min(pts[0].v_back, initial_v);

    for (size_t i = 1; i < n; ++i) {
        const size_t prev = i - 1;
        const size_t curr = i;
        const double ds = pts[curr].s - pts[prev].s;

        double accel_avail = limits_.max_lon_accel;

        // 摩擦椭圆耦合: 出弯加速时受横向附着力约束
        if (limits_.enable_friction_circle) {
            double v_prev = pts[prev].v_fwd;
            double a_y = v_prev * v_prev * std::abs(pts[prev].curvature);
            double ratio = a_y / limits_.max_lat_accel;
            if (ratio < 1.0) {
                accel_avail = limits_.max_lon_accel * std::sqrt(1.0 - ratio * ratio);
            } else {
                accel_avail = 0.1 * limits_.max_lon_accel;  // 保留最小加速余量
            }
        }

        // v_curr^2 <= v_prev^2 + 2 * a_accel * ds
        double max_accelerable_v = std::sqrt(pts[prev].v_fwd * pts[prev].v_fwd + 2.0 * accel_avail * ds);
        pts[curr].v_fwd = std::min(pts[curr].v_back, max_accelerable_v);
    }
}

}  // namespace velocity_profiler
