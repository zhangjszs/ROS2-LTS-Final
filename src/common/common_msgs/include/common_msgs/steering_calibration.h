#pragma once
// 前轮转角指令编码/解码的单一事实来源（Pure Pursuit / MPC / 仿真器 / 评测共用）。
// 语义约定：raw = neutral + steering_deg * units_per_degree，结果 clamp 到 [min_raw, max_raw]。
// 默认值为仿真器与 track_benchmark 既有协议（零位 90、1 raw/度、±25°）；
// 真实底盘协议尚未实测确认，确认后仅需以参数覆盖各字段，不允许再在各模块内散落硬编码。
// 详见 GitHub issue #2（统一转向编码与零位）与 #15（共用执行器适配层）。
#include <algorithm>
#include <cmath>

namespace common_msgs {
namespace vehicle {

struct SteeringCalibration {
    static constexpr double kPi = 3.14159265358979323846;

    double neutral{90.0};          // 零转角对应的 raw 指令值
    double units_per_degree{1.0};  // 每 1° 前轮转角对应的 raw 增量
    double min_raw{65.0};          // raw 下限（默认 -25°）
    double max_raw{115.0};         // raw 上限（默认 +25°）

    // 物理前轮转角 (rad) -> raw 指令值
    int encodeRad(double steering_rad) const {
        const double steering_deg = steering_rad * (180.0 / kPi);
        const double raw = neutral + steering_deg * units_per_degree;
        return static_cast<int>(std::lround(std::clamp(raw, min_raw, max_raw)));
    }

    // raw 指令值 -> 物理前轮转角 (rad)，先 clamp 再解码，保证往返一致
    double decodeRad(int steering_raw) const {
        const double clamped = std::clamp(static_cast<double>(steering_raw), min_raw, max_raw);
        return ((clamped - neutral) / units_per_degree) * (kPi / 180.0);
    }

    // 零转角 raw 值（停车/急停场景使用）
    int neutralRaw() const { return static_cast<int>(std::lround(neutral)); }
};

}  // namespace vehicle
}  // namespace common_msgs
