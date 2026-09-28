#pragma once
// 前轮转角指令编码/解码的单一事实来源（Pure Pursuit / MPC / 仿真器 / 评测共用）。
// 语义约定：raw = neutral + steering_deg * units_per_degree，结果 clamp 到 [min_raw, max_raw]。
// 默认值为仿真器与 track_benchmark 既有协议（零位 90、1 raw/度、±25°）；
// 真实底盘协议尚未实测确认，确认后仅需以参数覆盖各字段，不允许再在各模块内散落硬编码。
// 详见 GitHub issue #2（统一转向编码与零位）与 #15（共用执行器适配层）。
//
// 安全约束（#38，与 vehicle_command_codec.h 纵向侧对齐）：
//   * 非有限转角输入一律落零位（neutral），绝不产出 unspecified 值；
//   * 标定字段被误配（NaN / 上下限写反 / units≤0 / 超出字节宽度）时，不得触发
//     std::clamp 的前置条件违反（lo > hi 是 UB，在 libstdc++ 上是断言 abort），
//     而是先规整到安全域再编码；
//   * clamp-before-narrow：返回值保证落在 [0, 255]，调用方可直接窄化进 uint8；
//   * 误配不静默：isConfigValid() 与 encodeRadChecked().safe_fallback 供调用方告警与计数。
#include <algorithm>
#include <cmath>

namespace common_msgs {
namespace vehicle {

struct SteeringCalibration {
    static constexpr double kPi = 3.14159265358979323846;

    // raw 指令最终落在 uint8 里，因此合法量程必须处于字节宽度内（#38）。
    static constexpr double kByteMin = 0.0;
    static constexpr double kByteMax = 255.0;

    // 误配时的兜底协议：与仿真器/track_benchmark 既有默认值一致
    static constexpr double kFallbackNeutral = 90.0;
    static constexpr double kFallbackUnitsPerDegree = 1.0;
    static constexpr double kFallbackMinRaw = 65.0;
    static constexpr double kFallbackMaxRaw = 115.0;

    // 字段尾注一律放独立行：规避 clang-format v18(CI)/v21(本地) 的对齐分组差异。
    double neutral{kFallbackNeutral};  // 零转角对应的 raw 指令值

    double units_per_degree{kFallbackUnitsPerDegree};  // 每 1° 前轮转角对应的 raw 增量

    double min_raw{kFallbackMinRaw};  // raw 下限（默认 -25°）

    double max_raw{kFallbackMaxRaw};  // raw 上限（默认 +25°）

    // 一次配置自检：四字段有限、比例严格为正、上下限有序且落在字节宽度内、
    // 零位本身必须在量程内（否则“零转角”根本不可表达，#38）。
    // 节点侧应在启动时调用并在失败时明确告警，而不是依赖运行期的 clamp 行为。
    [[nodiscard]] bool isConfigValid() const noexcept {
        return std::isfinite(neutral) && std::isfinite(units_per_degree) && std::isfinite(min_raw) &&
               std::isfinite(max_raw) && units_per_degree > 0.0 && min_raw <= max_raw && min_raw >= kByteMin &&
               max_raw <= kByteMax && neutral >= min_raw && neutral <= max_raw;
    }

    struct SteeringCommand {
        int raw{};
        // true = 非有限输入或标定误配触发的安全降级（落零位）
        bool safe_fallback{};
    };

    // 物理前轮转角 (rad) -> raw 指令值。非有限输入落零位；配置误配时按规整后的安全域编码。
    [[nodiscard]] int encodeRad(double steering_rad) const noexcept { return encodeRadChecked(steering_rad).raw; }

    // 带降级标记的编码（与 ActuatorCalibration::ThrottleBrake 同构），供需要告警/计数的调用方使用。
    [[nodiscard]] SteeringCommand encodeRadChecked(double steering_rad) const noexcept {
        double lo = kFallbackMinRaw;
        double hi = kFallbackMaxRaw;
        sanitizedRange(lo, hi);
        if (!std::isfinite(steering_rad)) {
            return SteeringCommand{clampRounded(sanitizedNeutral(), lo, hi), true};
        }
        const double raw = sanitizedNeutral() + steering_rad * (180.0 / kPi) * sanitizedUnits();
        return SteeringCommand{clampRounded(raw, lo, hi), !isConfigValid()};
    }

    // raw 指令值 -> 物理前轮转角 (rad)，先 clamp 再换算（与旧行为逐值一致，且除零安全）。
    // 注：“越界 raw 本身是否是一条合法指令”不在本函数职责内，见 #39 的 decodeRadChecked。
    [[nodiscard]] double decodeRad(int steering_raw) const noexcept {
        double lo = kFallbackMinRaw;
        double hi = kFallbackMaxRaw;
        sanitizedRange(lo, hi);
        const double clamped = clampRounded(static_cast<double>(steering_raw), lo, hi);
        return ((clamped - sanitizedNeutral()) / sanitizedUnits()) * (kPi / 180.0);
    }

    // 零转角 raw 值（停车/急停场景使用）；neutral 误配时落安全域中点。
    [[nodiscard]] int neutralRaw() const noexcept {
        double lo = kFallbackMinRaw;
        double hi = kFallbackMaxRaw;
        sanitizedRange(lo, hi);
        return clampRounded(sanitizedNeutral(), lo, hi);
    }

   private:
    // 下列规整函数保证：输出有限、lo <= hi、量程在字节宽度内、units > 0，
    // 因此 std::clamp 永不会收到 lo > hi（#38 的直接成因）。
    [[nodiscard]] double sanitizedMin() const noexcept {
        if (!std::isfinite(min_raw)) {
            return kFallbackMinRaw;
        }
        return std::clamp(min_raw, kByteMin, kByteMax);
    }

    [[nodiscard]] double sanitizedMax() const noexcept {
        if (!std::isfinite(max_raw)) {
            return kFallbackMaxRaw;
        }
        return std::clamp(max_raw, kByteMin, kByteMax);
    }

    // 上下限写反 = 该配置不再携带可用信息 → 整对退回默认协议，而不是自作主张地交换。
    void sanitizedRange(double& lo, double& hi) const noexcept {
        lo = sanitizedMin();
        hi = sanitizedMax();
        if (lo > hi) {
            lo = kFallbackMinRaw;
            hi = kFallbackMaxRaw;
        }
    }

    [[nodiscard]] double sanitizedNeutral() const noexcept {
        double lo = kFallbackMinRaw;
        double hi = kFallbackMaxRaw;
        sanitizedRange(lo, hi);
        if (!std::isfinite(neutral)) {
            return 0.5 * (lo + hi);  // 零位不可用：取量程中点（默认协议下即 90 = 零转角）
        }
        return std::clamp(neutral, lo, hi);
    }

    [[nodiscard]] double sanitizedUnits() const noexcept {
        if (!std::isfinite(units_per_degree) || units_per_degree <= 0.0) {
            return kFallbackUnitsPerDegree;
        }
        return units_per_degree;
    }

    // clamp-before-narrow：先约束到 [lo,hi]（lo<=hi 由调用方保证），再四舍五入取整。
    [[nodiscard]] static int clampRounded(double value, double lo, double hi) noexcept {
        if (!std::isfinite(value)) {
            value = 0.5 * (lo + hi);
        }
        const long rounded = std::lround(std::clamp(value, lo, hi));
        return static_cast<int>(std::clamp(rounded, static_cast<long>(lo), static_cast<long>(hi)));
    }
};

}  // namespace vehicle
}  // namespace common_msgs
