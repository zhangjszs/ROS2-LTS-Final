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
    double cross_track_error{0.0};         // 垂直航迹误差 (m, 左正右负)
    double heading_error{0.0};             // 航向角误差 (rad)
    double lateral_accel_g{0.0};           // 参考曲率推算侧向加速度 (g) —— 非实测
    double measured_lateral_accel_g{0.0};  // 由状态航向变化率估计的侧向加速度 (g)
    double speed{0.0};                     // 实时车速 (m/s)
    double steering_angle{0.0};            // 前轮转角 (rad)
    double timestamp{0.0};                 // 时间戳 (s)
    bool out_of_bounds{false};             // 本样本是否越出合法走廊
    bool cone_contact{false};              // 本样本车辆外廓是否与某锥桶接触
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

    // —— #17 C：判据与可信性字段（一律追加于末尾，保持既有聚合初始化向后兼容）——
    std::string track_version{"unversioned"};             // 赛道/评测几何版本标识
    bool closed_circuit{false};                           // 仅闭合赛道计圈
    bool finished{false};                                 // 是否满足成功判据（由 runner 置位）
    bool timed_out{false};                                // 是否超过最大运行时长
    bool stopped{false};                                  // 是否中途停车/中止
    int valid_laps{0};                                    // 通过有序进度+方向校验的有效圈数
    double best_valid_lap_time_s{0.0};                    // 仅当存在有效圈时 >0；未完赛不产出
    int collision_events{0};                              // 外廓判定、按锥桶去重的碰撞事件数
    int collision_cones{0};                               // 被碰到的不同锥桶数
    int out_of_bounds_events{0};                          // 越出合法走廊事件数（进入越界的边沿计数）
    size_t out_of_bounds_samples{0};                      // 处于越界状态的样本数
    double max_abs_cross_track_m{0.0};                    // 最大横向偏差（含符号信息之外的绝对量）
    double peak_measured_lat_accel_g{0.0};                // 状态估计侧向加速度峰值
    double elapsed_s{0.0};                                // 总运行时长
    double total_length_m{0.0};                           // 单圈参考线长度
    std::string lat_accel_source{"reference_curvature"};  // lateral_accel_g/peak_lat_accel_g 来源标注
    std::string run_status{"running"};                    // running|finished|timeout|stopped|crashed
};

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
