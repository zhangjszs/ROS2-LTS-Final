#include "pure_pursuit/vehicle_command_encoder.h"

#include "vehicle_command_codec.h"  // #15：指令帧常量 + 校验和单一来源
#include "vehicle_command_frame.h"  // #31：线帧组装唯一拥有者

common_msgs::msg::HuatVehicleCmd VehicleCommandEncoder::encode(int steering, int brake_force, int pedal_ratio,
                                                               int gear_position, int working_mode, int racing_num,
                                                               int racing_status) const {
    // #31：意图组装仍归调用方（调用签名是公开接口），线帧拼装走 toMsg。
    common_msgs::vehicle::VehicleCommandRaw raw;
    raw.steering = static_cast<uint8_t>(steering);
    raw.brake_force = static_cast<uint8_t>(brake_force);
    raw.pedal_ratio = static_cast<uint8_t>(pedal_ratio);
    raw.gear_position = static_cast<uint8_t>(gear_position);
    raw.working_mode = static_cast<uint8_t>(working_mode);
    raw.racing_num = static_cast<uint8_t>(racing_num);
    raw.racing_status = static_cast<uint8_t>(racing_status);
    return common_msgs::vehicle::toMsg(raw);
}

common_msgs::msg::HuatVehicleCmd VehicleCommandEncoder::encodeBrake(int racing_num, int brake_force) const {
    return encode(neutral_steering_, brake_force, 0, 1, 1, racing_num, 4);
}

common_msgs::msg::HuatVehicleCmd VehicleCommandEncoder::encodeDrive(int steering, int pedal_ratio, int racing_num,
                                                                    int racing_status) const {
    return encode(steering, 0, pedal_ratio, 1, 1, racing_num, racing_status);
}

uint16_t VehicleCommandEncoder::computeChecksum(std::span<const uint8_t> payload) {
    uint32_t sum = 0;
    for (uint8_t byte : payload) {
        sum += byte;
    }
    return static_cast<uint16_t>(sum & 0xFFFF);
}

bool VehicleCommandEncoder::verifyChecksum(std::span<const uint8_t> payload, uint16_t expected_checksum) {
    return computeChecksum(payload) == expected_checksum;
}

uint16_t VehicleCommandEncoder::computeChecksum(const common_msgs::msg::HuatVehicleCmd& cmd) {
    // #15：委托共用编解码层的累加和（与 MPC/仿真解码同一来源，杜绝漂移）。
    common_msgs::vehicle::VehicleCommandRaw raw;
    raw.steering = cmd.steering;
    raw.brake_force = cmd.brake_force;
    raw.pedal_ratio = cmd.pedal_ratio;
    raw.gear_position = cmd.gear_position;
    raw.working_mode = cmd.working_mode;
    raw.racing_num = cmd.racing_num;
    raw.racing_status = cmd.racing_status;
    return common_msgs::vehicle::checksumRaw(raw);
}
