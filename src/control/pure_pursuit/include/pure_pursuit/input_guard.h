#ifndef PURE_PURSUIT_INPUT_GUARD_H
#define PURE_PURSUIT_INPUT_GUARD_H

#include <cstdint>
#include <rclcpp/time.hpp>
#include <span>

enum class GuardDecision { PROCEED, HARD_BRAKE, SOFT_BRAKE };

struct GuardResult {
    GuardDecision decision;
    // 制动指令字节：由构造时注入的标定值给出，本模块不再自带协议字面量（#15）。
    std::uint8_t brake_force;
    const char* reason;
};

class InputGuard {
   public:
    // hard/soft_brake_raw 来自 common_msgs::vehicle::ActuatorCalibration（#15）：
    // 急停与软停车两档分开，避免在守卫层散落 80/40 等底盘协议常量。
    InputGuard(double state_timeout, double path_timeout, std::uint8_t hard_brake_raw, std::uint8_t soft_brake_raw);

    // C++20 std::span 零拷贝接口：检查输入状态与路径点序列，决定是继续执行还是刹车
    GuardResult check(const rclcpp::Time& now, const rclcpp::Time& last_state_time, const rclcpp::Time& last_path_time,
                      std::span<const double> path_x, bool stop_requested, bool has_received_state,
                      bool has_received_path) const;

    // 检查输入状态，决定是继续执行还是刹车。
    // has_received_* 避免把 sim time t=0 误当成“从未收到”。
    GuardResult check(const rclcpp::Time& now, const rclcpp::Time& last_state_time, const rclcpp::Time& last_path_time,
                      bool path_empty, bool stop_requested, bool has_received_state, bool has_received_path) const;

   private:
    double state_timeout_;  // 状态超时时间
    double path_timeout_;   // 路径超时时间
    std::uint8_t hard_brake_raw_;
    std::uint8_t soft_brake_raw_;
};

#endif
