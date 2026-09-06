#ifndef PURE_PURSUIT_INPUT_GUARD_H
#define PURE_PURSUIT_INPUT_GUARD_H

#include <rclcpp/time.hpp>
#include <span>

enum class GuardDecision { PROCEED, HARD_BRAKE, SOFT_BRAKE };

struct GuardResult {
    GuardDecision decision;
    int brake_force;
    const char* reason;
};

class InputGuard {
   public:
    InputGuard(double state_timeout, double path_timeout);

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
};

#endif
