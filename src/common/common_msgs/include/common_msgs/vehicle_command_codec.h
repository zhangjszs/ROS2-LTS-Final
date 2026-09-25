#pragma once
// ============================================================================
// issue #15：底盘执行器指令编解码与纵向（油门/制动）标定的单一事实来源。
// 纯 std 依赖（不引入 rclcpp/ros），编码端（Pure Pursuit / MPC）与解码端
// （仿真器 / 评测）共用，杜绝协议拼装在校验和、帧头、字节窄化处散落漂移。
//
// 关键安全约束（对齐 ROS1-LTS-Final #6「负油门转 uint8 后成为 255」缺陷类）：
//   任何物理值在窄化到 uint8/字节之前，必须先经有限性检查并 clamp 到合法域；
//   非有限/缺失一律降级为「无油门 + 安全制动」，绝不因负值或溢出得到 255。
//
// 与 #2 的关系：SteeringCalibration 负责转角；本头负责纵向执行器与指令帧/校验和。
// 真实 VCU 协议/满量程/字节序尚未实车确认前，仅以参数覆盖各字段，禁止在模块内散落硬编码。
// 详见 docs/INTERFACE_CONTRACT.md §4。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace common_msgs {
namespace vehicle {

// —— 底盘指令帧常量（与仿真器/评测既有协议一致；实车 VCU 文档确认后仅改此处）——
inline constexpr uint8_t kCmdHead1 = 0xAA;
inline constexpr uint8_t kCmdHead2 = 0x55;
inline constexpr uint8_t kCmdLength = 10;

// 原始指令字节布局的纯数据视图（不含 ROS msg / 帧头 / 校验和），便于无 ROS 单测往返。
// 字段与 common_msgs/msg/HuatVehicleCmd 的指令区一一对应。
struct VehicleCommandRaw {
    uint8_t steering{};
    uint8_t brake_force{};
    uint8_t pedal_ratio{};
    uint8_t gear_position{1};
    uint8_t working_mode{1};
    uint8_t racing_num{};
    uint8_t racing_status{};
};

// 16 位累加和校验：指令区各字节求和取低 16 位（PP / MPC / 仿真解码一致）。
[[nodiscard]] inline uint16_t checksumRaw(const VehicleCommandRaw& r) noexcept {
    const unsigned sum = static_cast<unsigned>(r.steering) + r.brake_force + r.pedal_ratio + r.gear_position +
                         r.working_mode + r.racing_num + r.racing_status;
    return static_cast<uint16_t>(sum & 0xFFFFu);
}

// 帧头/长度合法性：单靠校验和不足以排除「整帧错位/帧头被截断」（帧头与 length 不参与累加和），
// 因此 #15 的帧有效性判定 = verifyFrame(...) && verifyChecksum(...)，由消费者（#16 仲裁）两者齐用。
[[nodiscard]] inline bool verifyFrame(std::uint8_t head1, std::uint8_t head2, std::uint8_t length) noexcept {
    return head1 == kCmdHead1 && head2 == kCmdHead2 && length == kCmdLength;
}

[[nodiscard]] inline bool verifyChecksum(const VehicleCommandRaw& r, uint16_t expected_checksum) noexcept {
    return checksumRaw(r) == expected_checksum;
}

// 纵向执行器标定：加速度 (m/s^2) <-> 油门/制动百分比 raw [0, pedal_full_scale]。
struct ActuatorCalibration {
    // 字段尾注均置于独立注释行：规避 clang-format v18/v21 对结构体字段尾注对齐的差异。
    // 正向满量程加速度（>0）
    double max_accel{5.0};
    // 制动满量程加速度幅值（>0）
    double max_decel{8.0};
    // 满油门/满制动对应 raw（默认 0-100；某些底盘为 0-255）
    double pedal_full_scale{100.0};
    // 急停/非有限输入时的锁定制动指令
    int emergency_brake_raw{80};
    // 软停车档制动指令（路径末端减速/路径缺失）：低于急停，与急停分开标定，
    // 不得共用 emergency_brake_raw（会改变现有减速行为）。
    int soft_brake_raw{40};
    // 标定版本；实车确认后递增，禁止隐式漂移
    std::string calibration_version{"sim-default-0"};

    struct ThrottleBrake {
        uint8_t pedal{};
        uint8_t brake{};
        // true 表示因非有限输入触发了安全降级
        bool safe_fallback{};
    };

    // 加速度 -> (油门, 制动)。互斥：正加速给油门、负加速给制动、零=滑行、非有限=安全制动。
    // 返回前逐字节已在 clamp 到 [0, min(pedal_full_scale,255)] 之后才窄化，绝不因负值/溢出得到 255。
    [[nodiscard]] ThrottleBrake encode(double accel_mps2) const {
        if (!std::isfinite(accel_mps2))
            return ThrottleBrake{0, clampByte(static_cast<double>(emergency_brake_raw)), true};
        if (accel_mps2 > 0.0) {
            const double pct = (accel_mps2 / positiveOr(max_accel, 1.0)) * pedal_full_scale;
            return ThrottleBrake{clampByte(pct), 0, false};
        }
        if (accel_mps2 < 0.0) {
            const double pct = ((-accel_mps2) / positiveOr(max_decel, 1.0)) * pedal_full_scale;
            return ThrottleBrake{0, clampByte(pct), false};
        }
        return ThrottleBrake{0, 0, false};  // 零加速度：滑行（油门、制动均为 0）
    }

    // raw (油门, 制动) -> 加速度；制动优先，均为正语义（与仿真既有解码一致）。
    [[nodiscard]] double decode(uint8_t pedal_ratio, uint8_t brake_force) const {
        const double scale = positiveOr(pedal_full_scale, 100.0);
        if (brake_force > 0)
            return -(static_cast<double>(brake_force) / scale) * positiveOr(max_decel, 1.0);
        return (static_cast<double>(pedal_ratio) / scale) * positiveOr(max_accel, 1.0);
    }

    // 急停锁定制动指令（clamp 后窄化，保证合法字节）。
    [[nodiscard]] uint8_t emergencyBrakeRaw() const { return clampByte(static_cast<double>(emergency_brake_raw)); }

    // 软停车制动指令（同走 clamp-before-narrow）。
    [[nodiscard]] uint8_t softBrakeRaw() const { return clampByte(static_cast<double>(soft_brake_raw)); }

   private:
    static double positiveOr(double v, double fallback) noexcept {
        return (std::isfinite(v) && v > 0.0) ? v : fallback;
    }
    // clamp-before-narrow：先约束到 [0, min(full_scale,255)]，再截断转 uint8；非有限输入按 0 处理。
    uint8_t clampByte(double pct) const {
        if (!std::isfinite(pct))
            pct = 0.0;
        const double hi = std::min(positiveOr(pedal_full_scale, 100.0), 255.0);
        const long v = static_cast<long>(std::clamp(pct, 0.0, hi));  // 截断，镜像既有 static_cast<int> 语义
        return static_cast<uint8_t>(std::clamp(v, 0L, 255L));
    }
};

}  // namespace vehicle
}  // namespace common_msgs
