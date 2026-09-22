#include "track_benchmark/kpi_evaluator.hpp"

#include <algorithm>
#include <format>
#include <numbers>
#include <sstream>
#include <string_view>

namespace benchmark {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kGravity = 9.80665;
constexpr double kLapCoverageFraction = 0.95;  // 累计正向弧长需达单圈长的比例才算完成一圈
constexpr double kMinLapDurationS = 5.0;       // 防止抖动/回退导致重复计圈的最短圈时

[[nodiscard]] inline double NormalizeAngle(double angle) noexcept {
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

// 最小 JSON 字符串转义（仅处理引号/反斜杠/控制字符）
[[nodiscard]] inline std::string JsonEscape(std::string_view s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char ch : s) {
        switch (ch) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += ch;
        }
    }
    return out;
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

    // #17 C
    cone_hit_.assign(cones_.size(), false);
    collision_events_ = 0;
    lap_progress_ = 0.0;
    prev_s_ = 0.0;
    have_prev_s_ = false;
    last_lap_time_ = 0.0;
    in_bounds_ = true;
    oob_events_ = 0;
    oob_samples_ = 0;
    max_abs_cte_ = 0.0;
    peak_measured_lat_g_ = 0.0;
    prev_speed_ = 0.0;
    prev_theta_ = 0.0;
    have_prev_state_ = false;
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
    size_t closest_idx = 0;
    const double cte = ComputeCrossTrackError(x, y, theta, &closest_idx);
    const double abs_cte = std::abs(cte);

    if (samples_count_ == 0) {
        lap_start_time_ = timestamp;
        prev_time_ = timestamp;
        prev_steering_ = steering_angle;
        have_prev_state_ = false;
        // in_bounds_ 保持 Reset 的 true，使首次越界也能触发一个边沿事件
        if (closed_ && !centerline_.empty()) {
            prev_s_ = centerline_[closest_idx].s;
            have_prev_s_ = true;
        }
    }

    double heading_err = 0.0;
    double ref_lat_g = 0.0;
    double meas_lat_g = 0.0;
    if (!centerline_.empty()) {
        heading_err = NormalizeAngle(theta - centerline_[closest_idx].theta);
        // 参考曲率推算侧向加速度 a_y = v^2 * kappa（标注为参考，非实测）
        ref_lat_g = (v * v * std::abs(centerline_[closest_idx].curvature)) / kGravity;
    }
    // 状态估计侧向加速度 a_y ≈ v * d(theta)/dt
    if (have_prev_state_) {
        double dtm = timestamp - prev_time_;
        if (dtm > 1e-4) {
            double theta_dot = NormalizeAngle(theta - prev_theta_) / dtm;
            meas_lat_g = std::abs(0.5 * (v + prev_speed_) * theta_dot) / kGravity;
        }
    }

    // 统计累加
    samples_count_++;
    sum_sq_error_ += cte * cte;
    sum_abs_error_ += abs_cte;
    max_error_ = std::max(max_error_, abs_cte);
    max_abs_cte_ = std::max(max_abs_cte_, abs_cte);
    peak_lat_g_ = std::max(peak_lat_g_, ref_lat_g);
    peak_measured_lat_g_ = std::max(peak_measured_lat_g_, meas_lat_g);
    max_speed_ = std::max(max_speed_, v);
    sum_speed_ += v;

    // 转向角抖动
    double dt = timestamp - prev_time_;
    if (dt > 1e-4) {
        double steer_rate = (steering_angle - prev_steering_) / dt;
        sum_steer_rate_sq_ += steer_rate * steer_rate;
    }

    // 走廊越界（事件边沿计数 + 样本计数）
    bool now_out = (abs_cte > corridor_half_width_);
    if (now_out) {
        oob_samples_++;
        if (in_bounds_) {
            oob_events_++;
        }
    }
    in_bounds_ = !now_out;

    // 有效圈：仅闭合赛道，按累计正向弧长进度判定（反向不会累计 -> 反向过线不计圈）
    if (closed_ && have_prev_s_ && !centerline_.empty()) {
        double s = centerline_[closest_idx].s;
        double d = s - prev_s_;
        if (total_length_ > 0.0) {
            if (d > 0.5 * total_length_)
                d -= total_length_;
            else if (d < -0.5 * total_length_)
                d += total_length_;
        }
        if (d > 0.0)
            lap_progress_ += d;
        prev_s_ = s;
        double lap_time = timestamp - lap_start_time_;
        if (total_length_ > 0.0 && lap_progress_ >= kLapCoverageFraction * total_length_ &&
            lap_time > kMinLapDurationS) {
            completed_laps_++;
            last_lap_time_ = lap_time;
            best_lap_time_ = std::min(best_lap_time_, lap_time);
            lap_start_time_ = timestamp;
            lap_progress_ = 0.0;
        }
    }

    // 撞桶：车辆外廓（含锥桶半径）判定 + 按锥桶去重的事件计数
    bool cone_contact = false;
    const double hx = 0.5 * vehicle_length_ + cone_radius_;
    const double hy = 0.5 * vehicle_width_ + cone_radius_;
    const double cs = std::cos(theta), sn = std::sin(theta);
    for (size_t i = 0; i < cones_.size(); ++i) {
        const double dx = cones_[i].x - x;
        const double dy = cones_[i].y - y;
        const double local_x = dx * cs + dy * sn;   // 车体系纵向
        const double local_y = -dx * sn + dy * cs;  // 车体系横向
        if (std::abs(local_x) <= hx && std::abs(local_y) <= hy) {
            cone_contact = true;
            if (i < cone_hit_.size() && !cone_hit_[i]) {
                cone_hit_[i] = true;
                collision_events_++;
            }
        }
    }
    cone_collisions_ = collision_events_;  // 兼容旧字段：现为去重后的碰撞事件数

    prev_time_ = timestamp;
    prev_steering_ = steering_angle;
    prev_speed_ = v;
    prev_theta_ = theta;
    have_prev_state_ = true;

    return KpiStepData{.cross_track_error = cte,
                       .heading_error = heading_err,
                       .lateral_accel_g = ref_lat_g,
                       .measured_lateral_accel_g = meas_lat_g,
                       .speed = v,
                       .steering_angle = steering_angle,
                       .timestamp = timestamp,
                       .out_of_bounds = now_out,
                       .cone_contact = cone_contact};
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
    // 未完赛不产出最佳圈速：仅当有有效圈时报告 best_lap
    const double valid_best = (completed_laps_ > 0 && best_lap_time_ < 1e8) ? best_lap_time_ : 0.0;
    summary.best_lap_time_s = valid_best;
    summary.best_valid_lap_time_s = valid_best;
    summary.completed_laps = completed_laps_;
    summary.valid_laps = completed_laps_;
    summary.cone_collisions = cone_collisions_;
    summary.steering_jerk = std::sqrt(sum_steer_rate_sq_ / static_cast<double>(samples_count_));

    // #17 C 可信性字段
    int distinct_cones = 0;
    for (char c : cone_hit_) {
        if (c)
            ++distinct_cones;
    }
    summary.closed_circuit = closed_;
    summary.track_version = track_version_;
    summary.collision_events = collision_events_;
    summary.collision_cones = distinct_cones;
    summary.out_of_bounds_events = oob_events_;
    summary.out_of_bounds_samples = oob_samples_;
    summary.max_abs_cross_track_m = max_abs_cte_;
    summary.peak_measured_lat_accel_g = peak_measured_lat_g_;
    summary.total_length_m = total_length_;
    summary.lat_accel_source = "reference_curvature";

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

std::string KpiEvaluator::GenerateJsonReport(const KpiSummary& s) {
    std::string j;
    j += "{\n";
    j += "  \"schema\": \"fsac.benchmark.kpi/v1\",\n";
    j += "  \"track_name\": \"" + JsonEscape(s.track_name) + "\",\n";
    j += "  \"track_version\": \"" + JsonEscape(s.track_version) + "\",\n";
    j += "  \"controller_name\": \"" + JsonEscape(s.controller_name) + "\",\n";
    j += "  \"run_status\": \"" + JsonEscape(s.run_status) + "\",\n";
    j += "  \"lat_accel_source\": \"" + JsonEscape(s.lat_accel_source) + "\",\n";
    j += "  \"closed_circuit\": " + std::string(s.closed_circuit ? "true" : "false") + ",\n";
    j += "  \"finished\": " + std::string(s.finished ? "true" : "false") + ",\n";
    j += "  \"timed_out\": " + std::string(s.timed_out ? "true" : "false") + ",\n";
    j += "  \"stopped\": " + std::string(s.stopped ? "true" : "false") + ",\n";
    j += "  \"valid_laps\": " + std::to_string(s.valid_laps) + ",\n";
    j += "  \"completed_laps\": " + std::to_string(s.completed_laps) + ",\n";
    j += std::format("  \"best_valid_lap_time_s\": {:.6},\n", s.best_valid_lap_time_s);
    j += std::format("  \"best_lap_time_s\": {:.6},\n", s.best_lap_time_s);
    j += std::format("  \"current_lap_time_s\": {:.6},\n", s.current_lap_time_s);
    j += std::format("  \"rmse_lateral_m\": {:.6},\n", s.rmse_lateral_m);
    j += std::format("  \"mean_lateral_error_m\": {:.6},\n", s.mean_lateral_error_m);
    j += std::format("  \"max_lateral_error_m\": {:.6},\n", s.max_lateral_error_m);
    j += std::format("  \"max_abs_cross_track_m\": {:.6},\n", s.max_abs_cross_track_m);
    j += std::format("  \"peak_lat_accel_g\": {:.6},\n", s.peak_lat_accel_g);
    j += std::format("  \"peak_measured_lat_accel_g\": {:.6},\n", s.peak_measured_lat_accel_g);
    j += std::format("  \"max_speed_mps\": {:.6},\n", s.max_speed_mps);
    j += std::format("  \"avg_speed_mps\": {:.6},\n", s.avg_speed_mps);
    j += std::format("  \"steering_jerk\": {:.6},\n", s.steering_jerk);
    j += std::format("  \"total_length_m\": {:.6},\n", s.total_length_m);
    j += "  \"collision_events\": " + std::to_string(s.collision_events) + ",\n";
    j += "  \"collision_cones\": " + std::to_string(s.collision_cones) + ",\n";
    j += "  \"cone_collisions\": " + std::to_string(s.cone_collisions) + ",\n";
    j += "  \"out_of_bounds_events\": " + std::to_string(s.out_of_bounds_events) + ",\n";
    j += "  \"out_of_bounds_samples\": " + std::to_string(s.out_of_bounds_samples) + ",\n";
    j += "  \"total_samples\": " + std::to_string(s.total_samples) + "\n";
    j += "}\n";
    return j;
}

}  // namespace benchmark
