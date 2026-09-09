#include "track_benchmark/kpi_evaluator.hpp"

#include <algorithm>
#include <format>
#include <numbers>
#include <sstream>

namespace benchmark {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kGravity = 9.80665;

[[nodiscard]] inline double NormalizeAngle(double angle) noexcept {
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

}  // namespace

void KpiEvaluator::Reset() {
    samples_count_ = 0;
    sum_sq_error_ = 0.0;
    sum_abs_error_ = 0.0;
    max_error_ = 0.0;
    peak_lat_g_ = 0.0;
    peak_lon_g_ = 0.0;
    max_speed_ = 0.0;
    sum_speed_ = 0.0;
    prev_steering_ = 0.0;
    prev_time_ = 0.0;
    sum_steer_rate_sq_ = 0.0;
    lap_start_time_ = 0.0;
    best_lap_time_ = 1e9;
    completed_laps_ = 0;
    in_start_zone_ = true;
    cone_collisions_ = 0;
}

double KpiEvaluator::ComputeCrossTrackError(double x, double y, [[maybe_unused]] double theta,
                                            size_t* closest_idx_out) const {
    if (centerline_.empty())
        return 0.0;

    size_t best_idx = 0;
    double min_dist_sq = 1e18;

    for (size_t i = 0; i < centerline_.size(); ++i) {
        double dx = x - centerline_[i].x;
        double dy = y - centerline_[i].y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            best_idx = i;
        }
    }

    if (closest_idx_out)
        *closest_idx_out = best_idx;

    const auto& ref = centerline_[best_idx];
    double dx = x - ref.x;
    double dy = y - ref.y;

    // Frenet 投影横向偏差: 垂直切线向左为正, 向右为负
    // 法向量 n = [-sin(theta), cos(theta)]
    double cross_track = -dx * std::sin(ref.theta) + dy * std::cos(ref.theta);
    return cross_track;
}

KpiStepData KpiEvaluator::Update(double x, double y, double theta, double v, double steering_angle, double timestamp) {
    if (samples_count_ == 0) {
        lap_start_time_ = timestamp;
        prev_time_ = timestamp;
        prev_steering_ = steering_angle;
    }

    size_t closest_idx = 0;
    const double cte = ComputeCrossTrackError(x, y, theta, &closest_idx);
    const double abs_cte = std::abs(cte);

    // 航向角偏差
    double heading_err = 0.0;
    double lat_accel_g = 0.0;
    if (!centerline_.empty()) {
        heading_err = NormalizeAngle(theta - centerline_[closest_idx].theta);
        // 侧向加速度 a_y = v^2 * kappa
        lat_accel_g = (v * v * std::abs(centerline_[closest_idx].curvature)) / kGravity;
    }

    // 统计累加
    samples_count_++;
    sum_sq_error_ += cte * cte;
    sum_abs_error_ += abs_cte;
    max_error_ = std::max(max_error_, abs_cte);
    peak_lat_g_ = std::max(peak_lat_g_, lat_accel_g);
    max_speed_ = std::max(max_speed_, v);
    sum_speed_ += v;

    // 转向角抖动与控制平滑度
    double dt = timestamp - prev_time_;
    if (dt > 1e-4) {
        double steer_rate = (steering_angle - prev_steering_) / dt;
        sum_steer_rate_sq_ += steer_rate * steer_rate;
    }
    prev_time_ = timestamp;
    prev_steering_ = steering_angle;

    // 圈速检测 (对于闭合赛道)
    if (!centerline_.empty() && centerline_.size() > 20) {
        double dist_to_start = std::hypot(x - centerline_.front().x, y - centerline_.front().y);
        if (dist_to_start < 3.0) {
            if (!in_start_zone_ && (timestamp - lap_start_time_) > 5.0) {
                // 完成一圈
                completed_laps_++;
                double lap_time = timestamp - lap_start_time_;
                best_lap_time_ = std::min(best_lap_time_, lap_time);
                lap_start_time_ = timestamp;
                in_start_zone_ = true;
            }
        } else {
            in_start_zone_ = false;
        }
    }

    // 撞桶违规检查
    for (const auto& cone : cones_) {
        double cd = std::hypot(x - cone.x, y - cone.y);
        if (cd < 0.35) {  // 赛车外缘碰触锥桶半径
            cone_collisions_++;
            break;
        }
    }

    return KpiStepData{.cross_track_error = cte,
                       .heading_error = heading_err,
                       .lateral_accel_g = lat_accel_g,
                       .speed = v,
                       .steering_angle = steering_angle,
                       .timestamp = timestamp};
}

KpiSummary KpiEvaluator::GetSummary() const {
    KpiSummary summary;
    summary.total_samples = samples_count_;
    if (samples_count_ == 0)
        return summary;

    summary.rmse_lateral_m = std::sqrt(sum_sq_error_ / static_cast<double>(samples_count_));
    summary.mean_lateral_error_m = sum_abs_error_ / static_cast<double>(samples_count_);
    summary.max_lateral_error_m = max_error_;
    summary.peak_lat_accel_g = peak_lat_g_;
    summary.max_speed_mps = max_speed_;
    summary.avg_speed_mps = sum_speed_ / static_cast<double>(samples_count_);
    summary.current_lap_time_s = prev_time_ - lap_start_time_;
    summary.best_lap_time_s = (best_lap_time_ < 1e8) ? best_lap_time_ : summary.current_lap_time_s;
    summary.completed_laps = completed_laps_;
    summary.cone_collisions = cone_collisions_;
    summary.steering_jerk = std::sqrt(sum_steer_rate_sq_ / static_cast<double>(samples_count_));

    return summary;
}

std::string KpiEvaluator::GenerateMarkdownReport(const KpiSummary& s) {
    std::stringstream ss;
    ss << "# 🏁 FSAC 控制算法基准性能评估报告 (Benchmark Report)\n\n";
    ss << "| 评估维度 | 指标名称 (Metric) | 测量值 (Value) | 达标参考 / 竞技意义 |\n";
    ss << "| :--- | :--- | :--- | :--- |\n";
    ss << std::format(
        "| **循迹精度** | 横向误差均方根 (Lateral RMSE) | **{:.3f} m** | 越小越贴近理想赛车线（顶级要求 < 0.15m） |\n",
        s.rmse_lateral_m);
    ss << std::format(
        "| **极限偏差** | 最大横向偏移 (Max Lateral Error) | **{:.3f} m** | 决定过弯是否推头擦碰外侧锥桶 |\n",
        s.max_lateral_error_m);
    ss << std::format("| **平均偏差** | 平均绝对误差 (MAE) | **{:.3f} m** | 全程稳态循迹中枢表现 |\n",
                      s.mean_lateral_error_m);
    ss << std::format(
        "| **动力极限** | 峰值侧向加速度 (Peak Lateral G) | **{:.2f} g** | 衡量控制器对轮胎抓地力的压榨程度 |\n",
        s.peak_lat_accel_g);
    ss << std::format(
        "| **车速表现** | 最高车速 / 平均车速 | **{:.1f} / {:.1f} km/h** | 决定赛事圈速与全场得分核心 |\n",
        s.max_speed_mps * 3.6, s.avg_speed_mps * 3.6);
    ss << std::format("| **圈速表现** | 最优圈速 (Best Lap Time) | **{:.2f} s** | 完成圈数: {} 圈 |\n",
                      s.best_lap_time_s, s.completed_laps);
    ss << std::format(
        "| **控制品质** | 转向抖动率 (Steering Smoothness) | **{:.2f} rad/s** | 越小说明转向越平顺，无高频震颤 |\n",
        s.steering_jerk);
    ss << std::format(
        "| **安全合规** | 撞桶惩罚计数 (Cone Collisions) | **{} 次** | 比赛每撞一个罚 2 秒，0 犯规为佳 |\n",
        s.cone_collisions);
    ss << "\n> 💡 *提示：本基准数据可直接用于后续 MPC 与 Pure Pursuit 的量化性能对比！*\n";
    return ss.str();
}

}  // namespace benchmark
