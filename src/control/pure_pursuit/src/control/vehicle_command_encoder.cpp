#include "pure_pursuit/vehicle_command_encoder.h"

#include "vehicle_command_codec.h"  // #15：指令帧常量 + 校验和单一来源

common_msgs::msg::HuatVehicleCmd VehicleCommandEncoder::encode(int steering, int brake_force, int pedal_ratio,
                                                               int gear_position, int working_mode, int racing_num,
                                                               int racing_status) const {
    common_msgs::msg::HuatVehicleCmd cmd;
    setHeader(cmd);
    cmd.steering = steering;
    cmd.brake_force = brake_force;
    cmd.pedal_ratio = pedal_ratio;
    cmd.gear_position = gear_position;
    cmd.working_mode = working_mode;
    cmd.racing_num = racing_num;
    cmd.racing_status = racing_status;
    cmd.checksum = computeChecksum(cmd);
    return cmd;
}

common_msgs::msg::HuatVehicleCmd VehicleCommandEncoder::encodeBrake(int racing_num, int brake_force) const {
    return encode(neutral_steering_, brake_force, 0, 1, 1, racing_num, 4);
}

common_msgs::msg::HuatVehicleCmd VehicleCommandEncoder::encodeDrive(int steering, int pedal_ratio, int racing_num,
                                                                    int racing_status) const {
    return encode(steering, 0, pedal_ratio, 1, 1, racing_num, racing_status);
}

void VehicleCommandEncoder::setHeader(common_msgs::msg::HuatVehicleCmd& cmd) {
    cmd.head1 = common_msgs::vehicle::kCmdHead1;
    cmd.head2 = common_msgs::vehicle::kCmdHead2;
    cmd.length = common_msgs::vehicle::kCmdLength;
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
