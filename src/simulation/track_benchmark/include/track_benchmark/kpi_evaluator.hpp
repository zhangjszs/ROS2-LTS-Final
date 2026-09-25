#pragma once

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "track_benchmark/track_generator.hpp"

namespace benchmark {

/**
 * @brief 实时运行步 KPI 数据帧
 */
struct KpiStepData {
    double cross_track_error{0.0};  // 垂直航迹误差 (m, 左正右负)
    double heading_error{0.0};      // 航向角误差 (rad)
    double lateral_accel_g{0.0};    // 参考曲率推算侧向加速度 (g)，非实测
    // #17 新增（字段行不带尾注，避免跨 clang-format 版本对齐差异）：
    double measured_lateral_accel_g{0.0};
    double speed{0.0};           // 实时车速 (m/s)
    double steering_angle{0.0};  // 前轮转角 (rad)
    double timestamp{0.0};       // 时间戳 (s)
    bool out_of_bounds{false};
    bool cone_contact{false};
};

/**
 * @brief 全局汇总评测 KPI 报告
 */
struct KpiSummary {
    std::string track_name{"Default"};
    std::string controller_name{"PurePursuit"};
    double rmse_lateral_m{0.0};        // 横向偏差均方根误差 (m)
    double max_lateral_error_m{0.0};   // 最大横向偏差 (m)
    double mean_lateral_error_m{0.0};  // 平均绝对偏差 (m)
    double peak_lat_accel_g{0.0};      // 峰值侧向加速度 (g)
    double peak_lon_accel_g{0.0};      // 峰值纵向加速度 (g)
    double max_speed_mps{0.0};         // 最高时速 (m/s)
    double avg_speed_mps{0.0};         // 平均时速 (m/s)
    double current_lap_time_s{0.0};    // 当前圈耗时 (s)
    double best_lap_time_s{0.0};       // 最优单圈圈速 (s)
    int completed_laps{0};             // 完赛圈数
    int cone_collisions{0};            // 撞桶违规次数
    double steering_jerk{0.0};         // 转向平滑度 / 抖动均方值
    size_t total_samples{0};           // 采样点数

    // —— #17 C：判据与可信性字段（追加于末尾保持聚合初始化向后兼容；字段行不带尾注，
    //    以免不同 clang-format 版本的 AlignTrailingComments 产生分歧）。含义：
    //    track_version 赛道/评测版本; closed_circuit 仅闭合赛道计圈; finished/timed_out/
    //    stopped 运行终态; valid_laps 有序进度+方向校验的有效圈; best_valid_lap_time_s 仅有效
    //    圈时>0; collision_events/cones 外廓去重碰撞; out_of_bounds_* 合法走廊越界;
    //    peak_measured_lat_accel_g 状态估计侧向加速度; lat_accel_source 侧向加速度来源。
    std::string track_version{"unversioned"};
    bool closed_circuit{false};
    bool finished{false};
    bool timed_out{false};
    bool stopped{false};
    int valid_laps{0};
    double best_valid_lap_time_s{0.0};
    int collision_events{0};
    int collision_cones{0};
    int out_of_bounds_events{0};
    size_t out_of_bounds_samples{0};
    double max_abs_cross_track_m{0.0};
    double peak_measured_lat_accel_g{0.0};
    double elapsed_s{0.0};
    double total_length_m{0.0};
    std::string lat_accel_source{"reference_curvature"};
    std::string run_status{"running"};
    // 沿参考线的**净带符号弧长进度** (m)：正向为正、反向为负。使“反向跑”不再只是
    // “有效圈=0”的缺失型证据，而是可正值识别的判据（#17 验收：反向经过起点需被识别）。
    double net_arc_progress_m{0.0};
};

/**
 * @brief #17 终态判定（闭环节点与离线 runner 共用同一判据，避免两处各写一套）。
 *
 * 超时优先；否则按有效圈数判完赛（require_laps<=0 时以“至少 1 个有效圈”为准）。
 * 该函数只把累计结果落成 finished/timed_out/run_status 三个终态字段，不改变任何累计量。
 * 说明：闭合赛道由 valid_laps 判定；直线赛不计圈，其“到终点完赛”语义由离线 runner 的
 * reached_end 负责 —— ROS 节点无法观测终点线，故直线赛在闭环里只会是 timeout/incomplete。
 */
inline void ApplyTerminalStatus(KpiSummary& s, bool timed_out, int require_laps) {
    if (timed_out) {
        s.timed_out = true;
        s.finished = false;
        s.run_status = "timeout";
        return;
    }
    const bool laps_ok = (require_laps > 0) ? (s.valid_laps >= require_laps) : (s.valid_laps > 0);
    s.finished = laps_ok;
    s.run_status = laps_ok ? "finished" : "incomplete";
}

/**
 * @brief 离线性能 KPI 评估引擎
 */
class KpiEvaluator {
   public:
    explicit KpiEvaluator(std::vector<CenterlinePoint> centerline = {}) : centerline_(std::move(centerline)) {}

    void SetCenterline(std::vector<CenterlinePoint> centerline) {
        centerline_ = std::move(centerline);
        Reset();
    }

    void SetTrackCones(std::vector<BenchmarkCone> cones) {
        cones_ = std::move(cones);
        cone_hit_.assign(cones_.size(), false);
    }

    /**
     * @brief 设置赛道几何与判据参数（闭合性、单圈长、合法走廊半宽、车辆外廓）。由 runner 从赛道定义传入。
     */
    void SetCircuitGeometry(double total_length, double corridor_half_width, bool closed_circuit,
                            double vehicle_length = 1.5, double vehicle_width = 1.0, double cone_radius = 0.10) {
        total_length_ = total_length;
        corridor_half_width_ = corridor_half_width;
        closed_ = closed_circuit;
        vehicle_length_ = vehicle_length;
        vehicle_width_ = vehicle_width;
        cone_radius_ = cone_radius;
    }

    /**
     * @brief 设置赛道/评测版本标识（写入报告，供回归比较区分口径）。
     */
    void SetTrackVersion(std::string version) { track_version_ = std::move(version); }

    /**
     * @brief 重置评估器统计状态
     */
    void Reset();

    /**
     * @brief 周期性输入车辆状态并累积更新评估指标
     */
    KpiStepData Update(double x, double y, double theta, double v, double steering_angle, double timestamp);

    /**
     * @brief 获取当前累计汇总的 KPI 指标
     */
    [[nodiscard]] KpiSummary GetSummary() const;

    /**
     * @brief 导出美观格式化的 Markdown 性能对比评测报告
     */
    [[nodiscard]] static std::string GenerateMarkdownReport(const KpiSummary& summary);

    /**
     * @brief 导出机读 JSON 结果（含判据/版本/状态字段），供 CI 回归与基线比较使用 (#17 A)
     */
    [[nodiscard]] static std::string GenerateJsonReport(const KpiSummary& summary);

    /**
     * @brief 计算车辆到参考中心线最近点的横向垂直偏差 e_y
     */
    [[nodiscard]] double ComputeCrossTrackError(double x, double y, [[maybe_unused]] double theta = 0.0,
                                                size_t* closest_idx_out = nullptr) const;

    [[nodiscard]] bool has_centerline() const noexcept { return !centerline_.empty(); }

   private:
    std::vector<CenterlinePoint> centerline_;
    std::vector<BenchmarkCone> cones_;

    // 统计累加器
    size_t samples_count_{0};
    double sum_sq_error_{0.0};
    double sum_abs_error_{0.0};
    double max_error_{0.0};
    double peak_lat_g_{0.0};
    double peak_lon_g_{0.0};
    double max_speed_{0.0};
    double sum_speed_{0.0};
    double prev_steering_{0.0};
    double prev_time_{0.0};
    double sum_steer_rate_sq_{0.0};

    // 圈速计时
    double lap_start_time_{0.0};
    double best_lap_time_{1e9};
    int completed_laps_{0};
    bool in_start_zone_{true};
    int cone_collisions_{0};

    // —— #17 C 状态 ——
    double total_length_{0.0};
    double corridor_half_width_{1.5};
    bool closed_{false};
    double vehicle_length_{1.5};
    double vehicle_width_{1.0};
    double cone_radius_{0.10};
    std::string track_version_{"unversioned"};

    // 进度计圈：沿参考线弧长的累计正向进度 + 方向校验
    double lap_progress_{0.0};  // 本圈累计正向弧长进度
    // 全程净带符号弧长进度（反向为负），仅供诊断/判据，不参与计圈
    double net_progress_{0.0};
    double prev_s_{0.0};
    bool have_prev_s_{false};
    double last_lap_time_{0.0};

    // 外廊碰撞去重
    std::vector<char> cone_hit_;
    int collision_events_{0};

    // 走廊越界
    bool in_bounds_{true};
    int oob_events_{0};
    size_t oob_samples_{0};
    double max_abs_cte_{0.0};

    // 侧向加速度来源区分（状态估计）
    double prev_speed_{0.0};
    double prev_theta_{0.0};
    double peak_measured_lat_g_{0.0};
    bool have_prev_state_{false};
};

}  // namespace benchmark
