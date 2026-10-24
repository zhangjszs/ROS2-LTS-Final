#pragma once
// ============================================================================
// issue #90：八字绕环（skidpad）完赛判据（纯 std，无 ROS 依赖）。
//
// 赛制：车辆经进场通道驶入，第一圆（y>0）逆时针绕 required_laps 圈，经换圆切点
// 过渡到第二圆（y<0）顺时针绕 required_laps 圈，最后经出场通道驶离——四段全部
// 完成才算完赛。本头把该判据固化为相位状态机 + 绕角累积计圈，供离线 runner /
// ROS 闭环门禁（#91）共用同一份判据——两处各写一套必然漂移（沿 #80 约定）。
//
// 计圈用绕角累积（signed winding angle）而非位置采样：对每圆圆心逐拍累加
// atan2 增量（wrap 到 (-pi, pi]），横向噪声在整圈尺度上近似抵消，抗 0.3m 级
// 扰动；而「第 N 次经过某采样点」式的计圈在噪声下必漂。
//
// 完赛门槛打折 lap_complete_fraction（默认 0.8）：一圈的绕角 2π，噪声在 288
// 拍上的角向随机游走 3σ ≈ 1.7 rad，满 2 圈的绕角 4π 必须留出该余量；0.8 仍
// 与「仅 1 圈」（绕角 ≈ 2π）拉开 >4 rad 的区分度。反向绕行累积的是错符号
// 绕角，永不达标——判据天然可证伪。
//
// 只观测、不干预（沿 #80）：回答「绕够了吗 / 换圆了吗 / 驶出了吗」，不产生
// 任何控制指令。参数全部来自赛道元数据（TrackDefinition::skidpad，#90），本
// 头不 include track_generator.hpp 以保持纯 std——spec 由调用方从元数据构造
// （离线 runner 沿 #81 finish_spec 的同一模式）。
// ============================================================================
#include <cmath>
#include <numbers>

namespace benchmark {

/// #90：八字绕环完赛判据参数。几何项全部来自赛道元数据，无硬编码；容差项带
/// 默认值，调用方按需覆盖。
struct SkidpadFinishSpec {
    bool enabled{false};         // 赛道是否具备八字完赛语义（非 skidpad 为 false）
    double right_center_x{0.0};  // 第一绕行圆（y>0）圆心 (m)
    double right_center_y{9.125};
    double left_center_x{0.0};  // 第二绕行圆（y<0）圆心 (m)
    double left_center_y{-9.125};
    double path_radius{9.125};          // 中心线绕圈半径 (m)
    int required_laps{2};               // 每圆要求圈数
    double exit_x{0.0};                 // 出场通道起点 X (m)
    double corridor_half_width{1.5};    // 出场通道 |y| 判定半宽 (m)
    double band_tol{1.0};               // 圆环带归属容差 (m)：点到圆心距离对 path_radius 的偏差上界
    double lap_complete_fraction{0.8};  // 完赛绕角门槛 = 2π·laps·fraction（噪声余量，见头注释）
};

/// #90：八字绕环完赛状态机（纯 std，只观测不干预）。
///
/// 相位：enter → right_lap → transition → left_lap → exit → finished。
/// 每个相位只认可一类证据（绕角 / 圆环带归属 / 出场通道），前序证据不足时后续
/// 观测不推进状态——「右圆少绕一圈、其余照走」这类轨迹会永远停在 right_lap。
class SkidpadFinishJudge {
   public:
    enum class Phase { kEnter, kRightLap, kTransition, kLeftLap, kExit, kFinished };

    explicit SkidpadFinishJudge(const SkidpadFinishSpec& spec) : spec_(spec) {
        valid_ =
            spec_.enabled && std::isfinite(spec_.right_center_x) && std::isfinite(spec_.right_center_y) &&
            std::isfinite(spec_.left_center_x) && std::isfinite(spec_.left_center_y) &&
            std::isfinite(spec_.path_radius) && std::isfinite(spec_.exit_x) && spec_.path_radius > 0.0 &&
            spec_.required_laps > 0 &&
            std::hypot(spec_.right_center_x - spec_.left_center_x, spec_.right_center_y - spec_.left_center_y) > 0.0;
    }

    /// 每拍喂入观测（位置 m）。非法 spec 或非有限输入拍被安全忽略（不推进、不 UB）。
    void Update(double x, double y) {
        if (!valid_ || phase_ == Phase::kFinished)
            return;
        if (!std::isfinite(x) || !std::isfinite(y))
            return;
        switch (phase_) {
            case Phase::kEnter:
                // 等待进入第一圆环带（进场通道对圆心的绕角不足以误触发，见头注释）。
                if (InBand(x, y, spec_.right_center_x, spec_.right_center_y)) {
                    Anchor(x, y, spec_.right_center_x, spec_.right_center_y);
                    phase_ = Phase::kRightLap;
                }
                break;
            case Phase::kRightLap:
                // 第一圆逆时针：绕角正向累积到门槛即视为绕满（换圆前的冗余绕行无害）。
                right_winding_ += DeltaWinding(x, y, spec_.right_center_x, spec_.right_center_y);
                if (right_winding_ >= LapThreshold())
                    phase_ = Phase::kTransition;
                break;
            case Phase::kTransition:
                // 等待进入第二圆环带。「跳过换圆直接出场」的轨迹即便误入本相位，
                // 后续对第二圆的绕角量级也远不足一圈门槛，判据仍可证伪。
                if (InBand(x, y, spec_.left_center_x, spec_.left_center_y)) {
                    Anchor(x, y, spec_.left_center_x, spec_.left_center_y);
                    phase_ = Phase::kLeftLap;
                }
                break;
            case Phase::kLeftLap:
                // 第二圆顺时针：绕角负向累积到门槛。
                left_winding_ += DeltaWinding(x, y, spec_.left_center_x, spec_.left_center_y);
                if (left_winding_ <= -LapThreshold())
                    phase_ = Phase::kExit;
                break;
            case Phase::kExit:
                // 绕满双圆后驶入出场通道（越过 exit_x 且横向回到通道内）→ 完赛。
                if (x >= spec_.exit_x && std::abs(y) <= spec_.corridor_half_width) {
                    phase_ = Phase::kFinished;
                }
                break;
            case Phase::kFinished:
                break;
        }
    }

    [[nodiscard]] bool IsFinished() const noexcept { return phase_ == Phase::kFinished; }
    [[nodiscard]] Phase phase() const noexcept { return phase_; }
    /// 观测字段（#92 数据面门禁断言「右圆绕角 ≥ 2π」用），单位 rad。
    [[nodiscard]] double right_winding() const noexcept { return right_winding_; }
    [[nodiscard]] double left_winding() const noexcept { return left_winding_; }
    /// spec 是否合法（非法：未启用 / NaN 几何 / 非正半径或圈数 / 两圆圆心重合）。
    /// 非法 spec 永不 finished。
    [[nodiscard]] bool valid() const noexcept { return valid_; }
    [[nodiscard]] const SkidpadFinishSpec& spec() const noexcept { return spec_; }

   private:
    // 环带归属：点到圆心距离落在 [radius - tol, radius + tol]。中心线绕圈半径即
    // 环带中心；tol 吸收横向跟踪偏差（≤ 噪声 + 半道宽量级）。
    [[nodiscard]] bool InBand(double x, double y, double cx, double cy) const noexcept {
        const double dist = std::hypot(x - cx, y - cy);
        return dist >= spec_.path_radius - spec_.band_tol && dist <= spec_.path_radius + spec_.band_tol;
    }
    // 进入新绕圈相位时锚定上一拍方位角（绕角累积起点）。
    void Anchor(double x, double y, double cx, double cy) noexcept { last_angle_ = std::atan2(y - cy, x - cx); }
    // 累加对圆心的方位角增量（wrap 到 (-pi, pi]）并滑动锚点。
    [[nodiscard]] double DeltaWinding(double x, double y, double cx, double cy) noexcept {
        const double a = std::atan2(y - cy, x - cx);
        double d = a - last_angle_;
        d = std::atan2(std::sin(d), std::cos(d));  // wrap：跨 ±pi 不产生 ±2pi 假增量
        last_angle_ = a;
        return d;
    }
    [[nodiscard]] double LapThreshold() const noexcept {
        return 2.0 * std::numbers::pi_v<double> * static_cast<double>(spec_.required_laps) *
               spec_.lap_complete_fraction;
    }

    SkidpadFinishSpec spec_;
    bool valid_{false};
    Phase phase_{Phase::kEnter};
    double last_angle_{0.0};
    double right_winding_{0.0};
    double left_winding_{0.0};
};

}  // namespace benchmark
