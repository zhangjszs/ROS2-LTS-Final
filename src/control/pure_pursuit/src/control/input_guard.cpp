#include "pure_pursuit/input_guard.h"

InputGuard::InputGuard(double state_timeout, double path_timeout, std::uint8_t hard_brake_raw,
                       std::uint8_t soft_brake_raw)
    : state_timeout_(state_timeout),
      path_timeout_(path_timeout),
      state_lease_(common_msgs::vehicle::LeaseConfig{state_timeout, 0.0}),
      path_lease_(common_msgs::vehicle::LeaseConfig{path_timeout, 0.0}),
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
    // #30：到达活性走租约（Absent=没来过，Stale=来过但超时，FromFuture=未来戳超容限；
    // 与台账矩阵第 3 行一致：超龄/未来戳一律拒收，不刷新行驶许可）。
    using common_msgs::vehicle::Freshness;
    if (has_received_state) {
        state_lease_.observe(last_state_time.seconds());
    }
    const Freshness state_fresh = state_lease_.check(now.seconds());
    if (state_fresh != Freshness::kFresh) {
        return {GuardDecision::HARD_BRAKE, hard_brake_raw_,
                state_fresh == Freshness::kAbsent ? "No vehicle state received yet"
                                                  : "Vehicle state timeout or future stamp"};
    }
    if (has_received_path) {
        path_lease_.observe(last_path_time.seconds());
    }
    const Freshness path_fresh = path_lease_.check(now.seconds());
    if (path_fresh == Freshness::kAbsent) {
        return {GuardDecision::SOFT_BRAKE, soft_brake_raw_, "No path received yet"};
    }
    if (path_empty) {
        return {GuardDecision::SOFT_BRAKE, soft_brake_raw_, "Path is empty"};
    }
    if (path_fresh != Freshness::kFresh) {
        // Stale 或未来戳：超龄/未来戳一律拒收（台账矩阵第 3 行）。
        return {GuardDecision::SOFT_BRAKE, soft_brake_raw_, "Path timeout or future stamp"};
    }
    return {GuardDecision::PROCEED, 0, ""};
}
