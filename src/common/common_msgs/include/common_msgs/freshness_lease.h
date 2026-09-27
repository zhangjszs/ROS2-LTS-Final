#pragma once
// ============================================================================
// issue #30：新鲜度/超时语义单一来源（纯 std，可进 core_standalone）。
//
// PP InputGuard、MPC 来源年龄门、safety_monitor 看门狗、仲裁器 stale 分类
// 原来各存一份 now-last_* 判断、两种时钟表示；本模块收敛语义：
//   - 首帧 arming：从未观测到样本 = Absent（不等同于陈旧），调用方据此区分
//     “还没来过”（首帧宽容/不刷新许可）与“来过但超时”（降级）。
//   - 陈旧 vs 缺席 vs 未来戳：Stale / Absent / FromFuture 三态，调用方映射
//     为自己的决策（制动/拒收/忽略），本模块不做决策。
// 时钟由调用方注入（double 秒），ROS/s im-time 差异不出本模块。
// ============================================================================
#include <cmath>
#include <limits>

namespace common_msgs {
namespace vehicle {

struct LeaseConfig {
    // 样本年龄超过即 Stale（秒）；<0 表示禁用年龄检查（恒 Fresh，非 Absent 时）。
    double timeout_sec = 0.5;
    // 样本戳超前 now 的容限（秒）；超限即 FromFuture；<0 按 0 处理。
    double future_tolerance_sec = 0.0;
};

enum class Freshness {
    kAbsent,      // 从未观测到样本
    kFresh,       // 样本在容限内
    kStale,       // 样本超龄（或检查被禁用时不出现）
    kFromFuture,  // 样本戳超前 now 超过容限
};

class FreshnessLease {
   public:
    explicit FreshnessLease(LeaseConfig config = LeaseConfig{}) : config_(config) {}

    // 记录最新样本戳（到达时刻或 header.stamp，调用方决定语义）。
    void observe(double sample_time_sec) { last_sample_sec_ = sample_time_sec; }

    [[nodiscard]] bool hasSample() const { return std::isfinite(last_sample_sec_); }

    // 距 now 的样本年龄；从未观测返回 +inf。
    [[nodiscard]] double age(double now_sec) const {
        if (!hasSample())
            return std::numeric_limits<double>::infinity();
        return now_sec - last_sample_sec_;
    }

    [[nodiscard]] Freshness check(double now_sec) const {
        if (!hasSample())
            return Freshness::kAbsent;
        const double future_tol = config_.future_tolerance_sec >= 0.0 ? config_.future_tolerance_sec : 0.0;
        if (last_sample_sec_ - now_sec > future_tol)
            return Freshness::kFromFuture;
        if (config_.timeout_sec < 0.0)
            return Freshness::kFresh;
        return age(now_sec) > config_.timeout_sec ? Freshness::kStale : Freshness::kFresh;
    }

   private:
    LeaseConfig config_;
    double last_sample_sec_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace vehicle
}  // namespace common_msgs
