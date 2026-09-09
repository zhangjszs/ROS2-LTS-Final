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
    double lateral_accel_g{0.0};    // 实时侧向加速度 (g)
    double speed{0.0};              // 实时车速 (m/s)
    double steering_angle{0.0};     // 前轮转角 (rad)
    double timestamp{0.0};          // 时间戳 (s)
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

    void SetTrackCones(std::vector<BenchmarkCone> cones) { cones_ = std::move(cones); }

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
};

}  // namespace benchmark
