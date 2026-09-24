#pragma once
// ============================================================================
// issue #22：把纯纵向油门控制律从 Pure Pursuit 节点回调抽到 core 边界。
// 纯 std 依赖（无 ROS context、可单测）；行为与原节点内联实现逐条件等价：
//   P+I 控制律 + 抗积分饱和 clamp + 低/高速边界 blend + 中段上限钳制 + 整数截断 + [min,max] 限幅。
// 积分状态（sum_error_）由本类持有；停车/急停场景由调用方 reset() 清零
// （等价于原节点散布的 `sum_error_ = 0.0;`）。
//
// 用 C++20 concept 对参数结构做鸭子类型约束，避免 core 依赖具体 ROS 参数类型、
// 也不复制字段（单一事实来源仍是 PurePursuitParams.algorithm.throttle）。
// ============================================================================
#include <algorithm>
#include <concepts>

namespace pp_core {

// 任何暴露下列字段且可作数值读取的参数类型均满足。
template <typename P>
concept ThrottleParamsLike = requires(const P& p) {
    { p.target_speed } -> std::convertible_to<double>;
    { p.pid_kp } -> std::convertible_to<double>;
    { p.pid_ki } -> std::convertible_to<double>;
    { p.pid_integral_max } -> std::convertible_to<double>;
    { p.speed_low_threshold } -> std::convertible_to<double>;
    { p.current_low_speed } -> std::convertible_to<double>;
    { p.speed_high_threshold } -> std::convertible_to<double>;
    { p.current_high_speed } -> std::convertible_to<double>;
    { p.current_clamp_max } -> std::convertible_to<double>;
    { p.pedal_min } -> std::convertible_to<double>;
    { p.pedal_max } -> std::convertible_to<double>;
    { p.speed_blend_zone } -> std::convertible_to<double>;
};

// 单输入（速度误差）PID + 速度边界 blend 的纵向油门控制器。
class ThrottleController {
   public:
    void reset() noexcept {
        sum_error_ = 0.0;
        long_error_ = 0.0;
        long_current_ = 0.0;
    }

    double integral() const noexcept { return sum_error_; }
    double lastError() const noexcept { return long_error_; }
    double lastCurrent() const noexcept { return long_current_; }

    // 计算一帧油门：返回已截断为整数并限幅到 [pedal_min, pedal_max] 的 pedal 指令。
    template <ThrottleParamsLike P>
    int update(double current_speed, const P& p) {
        const double v = current_speed;
        const double lo_hi = p.speed_low_threshold;
        const double hi_lo = p.speed_high_threshold;
        const double zone = (p.speed_blend_zone > 0.0) ? p.speed_blend_zone : 0.0;

        const double long_error = p.target_speed - v;
        long_error_ = long_error;

        // 仅在 [lo_hi-zone, hi_lo+zone] 过渡区间内累计积分，并做抗饱和 clamp。
        const bool freeze_integral = (v <= lo_hi - zone) || (v >= hi_lo + zone);
        if (!freeze_integral) {
            sum_error_ += long_error;
            const double imax = p.pid_integral_max;
            sum_error_ = std::max(-imax, std::min(imax, sum_error_));
        }
        double long_current = p.pid_kp * long_error + p.pid_ki * sum_error_;

        // 低速边界 blend：v<=lo_lo 固定为 current_low_speed；过渡带内线性插值。
        const double lo_lo = lo_hi - zone;
        if (v <= lo_lo) {
            long_current = p.current_low_speed;
        } else if (zone > 0.0 && v < lo_hi) {
            const double t = (v - lo_lo) / zone;
            long_current = (1.0 - t) * p.current_low_speed + t * long_current;
        }

        // 高速边界 blend：v>=hi_hi 固定为 current_high_speed；过渡带内线性插值。
        const double hi_hi = hi_lo + zone;
        if (v >= hi_hi) {
            long_current = p.current_high_speed;
        } else if (zone > 0.0 && v > hi_lo) {
            const double t = (v - hi_lo) / zone;
            long_current = (1.0 - t) * long_current + t * p.current_high_speed;
        }

        // 中段（lo_hi, hi_lo）上限钳制，避免巡航段油门过高。
        if (v > lo_hi && v < hi_lo && long_current > p.current_clamp_max) {
            long_current = p.current_clamp_max;
        }
        long_current_ = long_current;

        int pedal = static_cast<int>(long_current);  // 与原 (float)(int) 截断语义一致
        if (pedal < p.pedal_min)
            pedal = static_cast<int>(p.pedal_min);
        else if (pedal > p.pedal_max)
            pedal = static_cast<int>(p.pedal_max);
        return pedal;
    }

   private:
    double sum_error_ = 0.0;
    double long_error_ = 0.0;
    double long_current_ = 0.0;
};

}  // namespace pp_core
