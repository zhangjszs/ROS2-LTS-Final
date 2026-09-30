#pragma once

#include <cstdint>

namespace mpc {

/**
 * @brief 连续求解拒解看门狗（#50：把"控制器在持续拒解"从日志提升为机读状态）。
 *
 * 背景（docs/MPC_TUNING_FREEZE.md §7.4 对照实验）：求解持续失败在运行期不可见——
 * 节点只打一条 throttled WARN 然后保持上次指令衰减，既不进入任何状态出口，
 * 也没有连续失败上限；一旦触发就是"车不动且没人解释为什么"。
 *
 * 语义（与 safety_monitor 的看门狗刻意区分：那是"输入超时"，这是"我自己解不出来"）：
 *   - Record(true)  ：一次成功求解 ⇒ 连续计数清零，rejecting 解除（拒解已不再连续）；
 *   - Record(false) ：一次拒解 ⇒ 连续计数 +1；达到阈值后 rejecting 锁入；
 *   - threshold == 0 ：看门狗禁用（永不点亮），total 计数仍然累计。
 *
 * 纯 std、无 ROS、无时间源——可进 core_standalone（#40 覆盖审计）。
 */
class RejectWatchdog {
   public:
    static constexpr std::uint64_t kDefaultThreshold = 50;  // 连续 50 拍 @50Hz = 1s（#50 实现建议）

    explicit RejectWatchdog(std::uint64_t threshold = kDefaultThreshold) noexcept : threshold_(threshold) {}

    /// 阈值可配（N>0 生效；0 = 禁用）。改阈值不重放旧事件，只清 rejecting 锁。
    void SetThreshold(std::uint64_t threshold) noexcept {
        threshold_ = threshold;
        if (threshold_ == 0 || consecutive_ < threshold_) {
            rejecting_ = false;
        }
    }

    /// 记录一次求解结果：solve_ok=false 计一次拒解，true 清零连续计数。
    void Record(bool solve_ok) noexcept {
        if (solve_ok) {
            consecutive_ = 0;
            rejecting_ = false;
            return;
        }
        ++consecutive_;
        ++total_;
        if (threshold_ > 0 && consecutive_ >= threshold_) {
            rejecting_ = true;
        }
    }

    /// 当前是否处于"持续拒解"（连续失败达到阈值且尚未被一次成功求解打断）。
    [[nodiscard]] bool rejecting() const noexcept { return rejecting_; }

    /// 当前连续拒解拍数（一次成功即清零）。
    [[nodiscard]] std::uint64_t consecutive_failures() const noexcept { return consecutive_; }

    /// 自启动以来的累计拒解拍数（只增不减，供出口对账）。
    [[nodiscard]] std::uint64_t total_failures() const noexcept { return total_; }

    /// 生效阈值（0 = 禁用）。
    [[nodiscard]] std::uint64_t threshold() const noexcept { return threshold_; }

   private:
    std::uint64_t threshold_;
    std::uint64_t consecutive_{0};
    std::uint64_t total_{0};
    bool rejecting_{false};
};

}  // namespace mpc
