#ifndef PURE_PURSUIT_VEHICLE_COMMAND_ENCODER_H
#define PURE_PURSUIT_VEHICLE_COMMAND_ENCODER_H

#include <cstdint>

#include "common_msgs/msg/huat_vehicle_cmd.hpp"

class VehicleCommandEncoder {
   public:
    VehicleCommandEncoder() = default;

    common_msgs::msg::HuatVehicleCmd encode(int steering, int brake_force, int pedal_ratio, int gear_position,
                                        int working_mode, int racing_num, int racing_status) const;

    common_msgs::msg::HuatVehicleCmd encodeBrake(int racing_num, int brake_force) const;

    common_msgs::msg::HuatVehicleCmd encodeDrive(int steering, int pedal_ratio, int racing_num, int racing_status) const;

   private:
    static uint16_t computeChecksum(const common_msgs::msg::HuatVehicleCmd& cmd);
    static void setHeader(common_msgs::msg::HuatVehicleCmd& cmd);
};

#endif
