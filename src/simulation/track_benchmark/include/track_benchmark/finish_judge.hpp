#pragma once
// ============================================================================
// issue #80：直线加速赛完赛判据（纯 std，无 ROS 依赖）。
//
// 赛制（D-014 首个端到端赛制 = 直线加速）：车辆从静止起跑，越过终点线后必须
// 在终点后的停车缓冲区内**停稳**，才算完赛。本头把该判据固化为一个纯 std 的
// 状态机 + 一个由物理约束反推的到达速度上限，供离线 runner（本单）与 ROS
// 闭环门禁（#82）共用同一份判据——两处各写一套必然漂移。
//
// 关键约束（#80 验收第 2 条）：**终点线之前不得出现制动指令**。因此“越线后
// 全力制动”对到达速度有硬性物理要求——若以 v 冲过终点线，制动距离 v²/(2a)
// 必须 ≤ 可用缓冲距离，否则车必然冲过缓冲区末端（完赛失败）。本头由该约束
// 反推速度上限，并要求全程**恒定**施加（超过上限后不再加速，而不是先冲过头
// 再回头减速——后者本身就是一次终点线前的制动）。
//
// 坐标口径：终点线与缓冲区末端都在赛道的**纵向 X 坐标**上，取值来自赛道
// 元数据（TrackDefinition::finish_x / runout_end_m，见 #81），消费方不得
// 硬编码 75 / 100。
// ============================================================================
#include <cmath>

namespace benchmark {

/// #80：直线赛终点/停车区判据参数。全部来自赛道元数据与车辆参数，无硬编码。
struct FinishRunoutSpec {
    bool enabled{false};          // 赛道是否具备终点语义（闭合赛道为 false）
    double finish_x{0.0};         // 终点线 X 坐标 (m)
    double runout_end_m{0.0};     // 终点后停车缓冲区末端 X 坐标 (m)
    double brake_decel{9.0};      // 可用制动减速度上限 (m/s^2)，取自车辆参数
    double stop_speed_eps{0.05};  // 停稳速度阈值 (m/s)
    // 停车距离安全系数：吸收“加速度一阶滞后”带来的额外制动距离（实测约 v·τ ≈ 2 m）。
    // 取 0.75 → 上限速度下理论制动距离 18.75 m，实测停稳位置约 96.5 m（缓冲区余量 ~3.5 m）。
    double safety{0.75};
};

/// #80：由“越线后必须在缓冲区内停稳”反推的**到达速度上限** (m/s)。
///
/// 依据 v²/(2a) ≤ 可用缓冲距离；safety 留出执行器滞后（加速度一阶滞后）余量。
/// 返回 0 表示不限速（无终点语义，或缓冲距离非正）。
[[nodiscard]] inline double approachSpeedCap(const FinishRunoutSpec& spec) noexcept {
    if (!spec.enabled)
        return 0.0;
    const double runout = spec.runout_end_m - spec.finish_x;
    if (runout <= 0.0)
        return 0.0;
    return std::sqrt(2.0 * spec.brake_decel * runout * spec.safety);
}

/// #80：越线/停稳状态机（纯 std）。
///
/// 只观测、不干预：它回答“越线了吗 / 停稳了吗 / 冲出缓冲区了吗”，制动指令由
/// 调用方经既有控制链路（ControlCommand）发出——判据层不得直接改写车辆状态。
class FinishJudge {
   public:
    explicit FinishJudge(const FinishRunoutSpec& spec) : spec_(spec) {}

    /// 每拍喂入观测（纵向位置 m / 速度 m/s / 仿真时间 s）。返回 true = 本拍首次越线。
    bool Update(double x, [[maybe_unused]] double v, double t) {
        if (!spec_.enabled || crossed_)
            return false;
        if (x >= spec_.finish_x) {
            crossed_ = true;
            cross_time_s_ = t;
            return true;
        }
        return false;
    }

    /// 停稳判定：必须先越线、速度低于阈值、且仍在缓冲区内。
    [[nodiscard]] bool IsStopped(double x, double v) const noexcept {
        return crossed_ && v < spec_.stop_speed_eps && x <= spec_.runout_end_m;
    }

    /// 冲出缓冲区（越线后未停稳就滑过缓冲区末端）→ 本次运行失败。
    [[nodiscard]] bool ExceededRunout(double x) const noexcept { return crossed_ && x > spec_.runout_end_m; }

    [[nodiscard]] bool crossed() const noexcept { return crossed_; }
    [[nodiscard]] double cross_time_s() const noexcept { return cross_time_s_; }
    [[nodiscard]] const FinishRunoutSpec& spec() const noexcept { return spec_; }

   private:
    FinishRunoutSpec spec_;
    bool crossed_{false};
    double cross_time_s_{0.0};
};

}  // namespace benchmark
