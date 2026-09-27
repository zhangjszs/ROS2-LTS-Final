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

}  // namespace vehicle
}  // namespace common_msgs
