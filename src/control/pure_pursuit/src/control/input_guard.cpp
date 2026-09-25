#include "pure_pursuit/input_guard.h"

InputGuard::InputGuard(double state_timeout, double path_timeout, std::uint8_t hard_brake_raw,
                       std::uint8_t soft_brake_raw)
    : state_timeout_(state_timeout),
      path_timeout_(path_timeout),
      hard_brake_raw_(hard_brake_raw),
      soft_brake_raw_(soft_brake_raw) {}

GuardResult InputGuard::check(const rclcpp::Time& now, const rclcpp::Time& last_state_time,
                              const rclcpp::Time& last_path_time, std::span<const double> path_x, bool stop_requested,
                              bool has_received_state, bool has_received_path) const {
    return check(now, last_state_time, last_path_time, path_x.empty(), stop_requested, has_received_state,
                 has_received_path);
}

GuardResult InputGuard::check(const rclcpp::Time& now, const rclcpp::Time& last_state_time,
                              const rclcpp::Time& last_path_time, bool path_empty, bool stop_requested,
                              bool has_received_state, bool has_received_path) const {
    if (stop_requested) {
        return {GuardDecision::HARD_BRAKE, hard_brake_raw_, "Emergency stop signal active"};
    }
    if (!has_received_state) {
        return {GuardDecision::HARD_BRAKE, hard_brake_raw_, "No vehicle state received yet"};
    }
    if ((now - last_state_time).seconds() > state_timeout_) {
        return {GuardDecision::HARD_BRAKE, hard_brake_raw_, "Vehicle state timeout"};
    }
    if (!has_received_path) {
        return {GuardDecision::SOFT_BRAKE, soft_brake_raw_, "No path received yet"};
    }
    if (path_empty) {
        return {GuardDecision::SOFT_BRAKE, soft_brake_raw_, "Path is empty"};
    }
    if ((now - last_path_time).seconds() > path_timeout_) {
        return {GuardDecision::SOFT_BRAKE, soft_brake_raw_, "Path timeout"};
    }
    return {GuardDecision::PROCEED, 0, ""};
}
