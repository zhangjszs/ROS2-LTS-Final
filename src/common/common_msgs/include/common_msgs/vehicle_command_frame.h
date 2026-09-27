#pragma once
// ============================================================================
// issue #31：指令帧唯一拥有者——物理意图进，线帧出。
//
// 纯数据层（vehicle_command_codec.h）保持无 ROS，可进 core_standalone；
// 本头是 ROS 侧的薄装配：PP / MPC / 仲裁器三处生产者统一经 toMsg 组帧，
// 节点内不再手写帧头常量与字段拷贝。帧格式变更只改此处。
// ============================================================================
#include "common_msgs/msg/huat_vehicle_cmd.hpp"
#include "vehicle_command_codec.h"

namespace common_msgs {
namespace vehicle {

// 物理意图（VehicleCommandRaw）→ 线帧（含帧头/长度/校验和）。
[[nodiscard]] inline common_msgs::msg::HuatVehicleCmd toMsg(const VehicleCommandRaw& raw) {
    common_msgs::msg::HuatVehicleCmd msg;
    msg.head1 = kCmdHead1;
    msg.head2 = kCmdHead2;
    msg.length = kCmdLength;
    msg.steering = raw.steering;
    msg.brake_force = raw.brake_force;
    msg.pedal_ratio = raw.pedal_ratio;
    msg.gear_position = raw.gear_position;
    msg.working_mode = raw.working_mode;
    msg.racing_num = raw.racing_num;
    msg.racing_status = raw.racing_status;
    msg.checksum = checksumRaw(raw);
    return msg;
}

// 安全停车意图工厂：零油门 + 制动字节 + 停车状态字。全零≠安全，禁止手写停车帧。
[[nodiscard]] inline VehicleCommandRaw makeSafeStopRaw(uint8_t steering_raw, uint8_t brake_raw, uint8_t racing_num) {
    VehicleCommandRaw raw;
    raw.steering = steering_raw;
    raw.brake_force = brake_raw;
    raw.pedal_ratio = 0;
    raw.racing_num = racing_num;
    raw.racing_status = 4;
    return raw;
}

}  // namespace vehicle
}  // namespace common_msgs
