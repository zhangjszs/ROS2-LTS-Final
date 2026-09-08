#ifndef PURE_PURSUIT_VEHICLE_COMMAND_ENCODER_H
#define PURE_PURSUIT_VEHICLE_COMMAND_ENCODER_H

#include <cstdint>
#include <span>

#include "common_msgs/msg/huat_vehicle_cmd.hpp"

class VehicleCommandEncoder {
   public:
    VehicleCommandEncoder() = default;

    common_msgs::msg::HuatVehicleCmd encode(int steering, int brake_force, int pedal_ratio, int gear_position,
                                            int working_mode, int racing_num, int racing_status) const;

    common_msgs::msg::HuatVehicleCmd encodeBrake(int racing_num, int brake_force) const;

    common_msgs::msg::HuatVehicleCmd encodeDrive(int steering, int pedal_ratio, int racing_num,
                                                 int racing_status) const;

    // C++20 std::span 接口：对通用字节缓冲区计算 16 位累加和校验码（支持任意连续容器与子切片）
    static uint16_t computeChecksum(std::span<const uint8_t> payload);

    // 校验接收到的二进制负载是否匹配
    static bool verifyChecksum(std::span<const uint8_t> payload, uint16_t expected_checksum);

    // 计算 HuatVehicleCmd 控制指令的校验码
    static uint16_t computeChecksum(const common_msgs::msg::HuatVehicleCmd& cmd);

   private:
    static void setHeader(common_msgs::msg::HuatVehicleCmd& cmd);
};

#endif
