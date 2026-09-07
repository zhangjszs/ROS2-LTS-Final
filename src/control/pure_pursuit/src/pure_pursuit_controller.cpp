#include "pure_pursuit/pure_pursuit_controller.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>
#include <ranges>
#include <utility>
#include <vector>

#include "pure_pursuit/pp_math.h"

using std::vector;
constexpr double kPi = std::numbers::pi_v<double>;

PurePursuitController::PurePursuitController(rclcpp::Node::SharedPtr node)
    : node_(node), params_(node), input_guard_(params_.safety.state_timeout, params_.safety.path_timeout) {
    steering_ = 90;
    pedal_ratio_ = 0;
    racing_num_ = params_.system.racing_num;
    racing_status_ = 1;

    pub_finall_cmd_ = node_->create_publisher<common_msgs::msg::HuatVehicleCmd>(params_.topics.vehicle_command, 1);
    latency_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(params_.topics.latency, 10);
    dropped_commands_pub_ = node_->create_publisher<std_msgs::msg::UInt64>(params_.topics.dropped_commands, 10);
    sub_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
        params_.topics.vehicle_state, 1,
        std::bind(&PurePursuitController::OnCarStateMessage, this, std::placeholders::_1));
    sub_path_ = node_->create_subscription<common_msgs::msg::HuatPathLimits>(
        params_.topics.path, 1,
        std::bind(&PurePursuitController::OnPathLimitsMessage, this, std::placeholders::_1));
    sub_stop_ = node_->create_subscription<common_msgs::msg::HuatStop>(
        params_.topics.stop, 1,
        std::bind(&PurePursuitController::OnStopMessage, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
                "[pure_pursuit] Topics: state=%s path=%s stop=%s cmd=%s latency=%s rate=%.1fHz startup_delay=%.2fs",
                params_.topics.vehicle_state.c_str(), params_.topics.path.c_str(), params_.topics.stop.c_str(),
                params_.topics.vehicle_command.c_str(), params_.topics.latency.c_str(), params_.system.control_rate,
                params_.system.startup_delay);

    diag_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
    diag_updater_->add("Pure Pursuit Health", this, &PurePursuitController::DiagnoseHealth);
    diag_updater_->setHardwareID("pure_pursuit");
}

void PurePursuitController::DiagnoseHealth(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    double latency_ms = last_latency_ * 1000;
    if (last_latency_ < 0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No path received yet");
    } else if (last_latency_ > 0.1) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "High latency");
    } else if (stop_requested_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Emergency stop active");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
    }
    stat.add("Latency (ms)", latency_ms);
    stat.add("Current speed", current_speed_);
    stat.add("Steering", steering_);
    stat.add("Path mode", path_mode_);
    stat.add("Stop requested", stop_requested_);
}

void PurePursuitController::OnPathLimitsMessage(const common_msgs::msg::HuatPathLimits::ConstSharedPtr &msgs) {
    if (not this->localTfValid_) {
        RCLCPP_WARN(node_->get_logger(), "[pure_pursuit] Vehicle state not received");
        return;
    }
    if (msgs->path.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[pure_pursuit] Received empty path, clearing reference");
        refx_.clear();
        refy_.clear();
        last_goal_idx_ = -1;
        return;
    }
    path_in_base_link_ = pp_math::isBaseLinkFrame(msgs->header.frame_id);
    has_received_path_ = true;
    refx_.clear();
    refy_.clear();
    last_goal_idx_ = -1;
    for (const auto &pt : msgs->path) {
        refx_.push_back(pt.x);
        refy_.push_back(pt.y);
    }
    path_mode_++;
    RCLCPP_DEBUG(node_->get_logger(), "[pure_pursuit] Received path with %zu points", msgs->path.size());

    rclcpp::Time now = node_->now();
    last_path_time_ = now;
    double stamp_age = (now - msgs->header.stamp).seconds();
    if (stamp_age > 10.0) {
        if (last_path_time_prev_.nanoseconds() / 1e9 > 0) {
            last_latency_ = (now - last_path_time_prev_).seconds();
        }
        last_path_time_prev_ = now;
        RCLCPP_DEBUG(node_->get_logger(), "[pure_pursuit] Pipeline interval: %.1f ms (rosbag mode)", last_latency_ * 1000);
    } else {
        last_latency_ = stamp_age;
        last_path_time_prev_ = now;
        if (last_latency_ > 0.1) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[pure_pursuit] High latency: %.0f ms (threshold: 100 ms)", last_latency_ * 1000);
        } else {
            RCLCPP_DEBUG(node_->get_logger(), "[pure_pursuit] Latency: %.1f ms", last_latency_ * 1000);
        }
    }

    std_msgs::msg::Float64MultiArray arr;
    arr.data = {last_latency_ * 1000.0};
    latency_pub_->publish(arr);
}

void PurePursuitController::OnCarStateMessage(const common_msgs::msg::HuatCarstate::ConstSharedPtr &msgs) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = msgs->car_state.x;
    pose.position.y = msgs->car_state.y;
    pose.position.z = 0;
    tf2::Quaternion qAux;
    qAux.setRPY(0.0, 0.0, msgs->car_state.theta);
    pose.orientation = tf2::toMsg(qAux);
    tf2::fromMsg(pose, this->localTf_);
    this->localTf_ = this->localTf_.inverse();
    this->localTfValid_ = true;

    current_x_ = msgs->car_state.x;
    current_y_ = msgs->car_state.y;
    current_speed_ = msgs->v;
    last_state_time_ = node_->now();
    has_received_state_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "[pure_pursuit] Current position: x=%f y=%f", current_x_, current_y_);
}

void PurePursuitController::OnStopMessage(const common_msgs::msg::HuatStop::ConstSharedPtr &msgs) {
    if (msgs->stop && !stop_requested_) {
        RCLCPP_WARN(node_->get_logger(), "[pure_pursuit] Stop signal received, executing emergency brake");
        stop_requested_ = true;
    } else if (!msgs->stop && stop_requested_) {
        RCLCPP_INFO(node_->get_logger(), "[pure_pursuit] Stop signal cleared, resuming control");
        stop_requested_ = false;
    }
}

double PurePursuitController::EstimatePathCurvature(int idx) const {
    return pp_math::estimateCurvature(refx_, refy_, idx);
}

int PurePursuitController::GetGoalIndex() {
    if (refx_.empty())
        return -1;

    const int n = static_cast<int>(refx_.size());
    const int base_window = params_.algorithm.path_search.search_window;
    const int max_window = params_.algorithm.path_search.search_window_max;
    const int backtrack = params_.algorithm.path_search.backtrack_window;
    int kappa_idx = pp_math::clampKappaIdx(last_goal_idx_ < 0 ? 1 : last_goal_idx_, n);
    double kappa_cur = EstimatePathCurvature(kappa_idx);
    double blend = std::min(1.0, kappa_cur / 0.5);
    int window = static_cast<int>(base_window + blend * (max_window - base_window));
    const int search_start = (last_goal_idx_ < 0) ? 0 : std::max(0, last_goal_idx_ - backtrack);
    const int search_end = (last_goal_idx_ < 0) ? n : std::min(last_goal_idx_ + window, n);
    const double cx = path_in_base_link_ ? 0.0 : current_x_;
    const double cy = path_in_base_link_ ? 0.0 : current_y_;

    pp_math::NearestIndexResult nearest = pp_math::findNearestIndex(refx_, refy_, cx, cy, search_start, search_end);
    if (nearest.idx < 0)
        return -1;

    const double max_ct = params_.algorithm.path_search.max_crosstrack_m;
    if (max_ct > 0.0 && nearest.dist_sq > max_ct * max_ct) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[pure_pursuit] Crosstrack %.2fm exceeds limit %.2fm, refusing goal",
                          std::sqrt(nearest.dist_sq), max_ct);
        return -1;
    }
    last_goal_idx_ = nearest.idx;
    return nearest.idx;
}

int PurePursuitController::GetLookaheadIndices(int current_idx, double lookahead, std::span<const double> refx,
                                               std::span<const double> refy) {
    if (current_idx < 0 || refx.empty() || refx.size() != refy.size()) {
        return 0;
    }
    const int n = static_cast<int>(refx.size());
    double distance_sum = 0.0;
    int idx = current_idx;

    // C++20 std::ranges 管道流水线：iota 惰性生成线段下标，transform 惰性计算步进欧氏距离
    auto segment_dists = std::views::iota(current_idx, std::max(current_idx, n - 1))
        | std::views::transform([&](int i) {
            double dx = refx[i + 1] - refx[i];
            double dy = refy[i + 1] - refy[i];
            return std::make_pair(i + 1, std::hypot(dx, dy));
        });

    for (const auto &[next_idx, seg_dist] : segment_dists) {
        if (distance_sum + seg_dist <= lookahead) {
            distance_sum += seg_dist;
            idx = next_idx;
        } else {
            break;
        }
    }

    if (idx >= 0 && idx < n)
        RCLCPP_DEBUG(node_->get_logger(), "[pure_pursuit] Lookahead point: x=%f y=%f", refx[idx], refy[idx]);
    return idx;
}

void PurePursuitController::ComputeControlCommand(common_msgs::msg::HuatControlCommand &cmd,
                                                  common_msgs::msg::HuatVehicleCmd &finall_cmd) {
    rclcpp::Time now = node_->now();
    GuardResult guard = input_guard_.check(now, last_state_time_, last_path_time_, refx_, stop_requested_,
                                           has_received_state_, has_received_path_);
    if (guard.decision != GuardDecision::PROCEED) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[pure_pursuit] %s, braking", guard.reason);
        sum_error_ = 0.0;
        filtered_angle_ = 0.0;
        finall_cmd = encoder_.encodeBrake(racing_num_, guard.brake_force);
        pub_finall_cmd_->publish(finall_cmd);
        dropped_commands_++;
        std_msgs::msg::UInt64 msg;
        msg.data = dropped_commands_;
        dropped_commands_pub_->publish(msg);
        return;
    }
    RCLCPP_DEBUG(node_->get_logger(), "[pure_pursuit] Current path mode: %d", path_mode_);
    const auto &steer = params_.algorithm.steering;
    const auto &throt = params_.algorithm.throttle;
    float delta_max = steer.delta_max;
    int goal_idx = GetGoalIndex();
    const int path_len = static_cast<int>(refx_.size());
    if (goal_idx < 0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[pure_pursuit] No valid goal index, hard braking");
        sum_error_ = 0.0;
        filtered_angle_ = 0.0;
        finall_cmd = encoder_.encodeBrake(racing_num_, 80);
        pub_finall_cmd_->publish(finall_cmd);
        return;
    }
    if (goal_idx >= 0 && path_len - goal_idx <= params_.algorithm.path_search.end_decel_points) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[pure_pursuit] Approaching path end (%d/%d), braking", goal_idx, path_len);
        sum_error_ = 0.0;
        filtered_angle_ = 0.0;
        finall_cmd = encoder_.encodeBrake(racing_num_, 40);
        pub_finall_cmd_->publish(finall_cmd);
        return;
    }
    int kappa_idx = pp_math::clampKappaIdx(goal_idx < 1 ? 1 : goal_idx, path_len);
    double kappa = (path_len >= 3) ? EstimatePathCurvature(kappa_idx) : 0.0;
    double adaptive_lookahead = steer.lookahead_base + steer.lookahead_speed_gain * std::max(0.0, current_speed_) -
                                steer.lookahead_curvature_gain * kappa;
    adaptive_lookahead = std::max(steer.lookahead_min, adaptive_lookahead);
    int lookahead_idx = GetLookaheadIndices(goal_idx, adaptive_lookahead, refx_, refy_);
    if (lookahead_idx >= 0 && lookahead_idx < path_len && lookahead_idx < static_cast<int>(refy_.size())) {
        Eigen::Vector3d product = path_in_base_link_
                                      ? Eigen::Vector3d(refx_[lookahead_idx], refy_[lookahead_idx], 0.0)
                                      : localTf_ * Eigen::Vector3d(refx_[lookahead_idx], refy_[lookahead_idx], 0.0);
        double goalX = product.x();
        double goalY = product.y();
        float alpha = std::atan2(static_cast<float>(goalY), static_cast<float>(goalX));
        alpha = (alpha > kPi) ? (alpha - 2 * kPi) : (alpha < -kPi) ? (alpha + 2 * kPi) : alpha;

        float delta = std::atan2(static_cast<float>(steer.pure_pursuit_gain * std::sin(alpha) / adaptive_lookahead), 1.0f);
        delta = std::max(std::min(delta_max, delta), -delta_max);
        if (std::abs(delta - filtered_angle_) > steer.filter_threshold) {
            delta = static_cast<float>(delta * steer.filter_blend_ratio + filtered_angle_ * (1.0 - steer.filter_blend_ratio));
        }
        filtered_angle_ = delta;
        cmd.steering_angle.data = delta;
        RCLCPP_DEBUG(node_->get_logger(), "[pure_pursuit] Steering angle: %f, steering: %d, speed: %f", delta, steering_, current_speed_);
        steering_ = static_cast<int>(cmd.steering_angle.data * 180 / kPi * steer.mapping.deg_per_rad) + steer.mapping.center_offset;
        long_error_ = throt.target_speed - current_speed_;
        {
            const double zone = throt.speed_blend_zone > 0.0 ? throt.speed_blend_zone : 0.0;
            const double v = current_speed_;
            const double lo_hi_pre = static_cast<double>(throt.speed_low_threshold);
            const double hi_lo_pre = static_cast<double>(throt.speed_high_threshold);
            const bool freeze_integral = (v <= lo_hi_pre - zone) || (v >= hi_lo_pre + zone);
            if (!freeze_integral) {
                sum_error_ += long_error_;
                sum_error_ = std::max(-throt.pid_integral_max, std::min(throt.pid_integral_max, sum_error_));
            }
        }
        long_current_ = throt.pid_kp * long_error_ + throt.pid_ki * sum_error_;

        {
            const double zone = throt.speed_blend_zone > 0.0 ? throt.speed_blend_zone : 0.0;
            const double v = current_speed_;

            double lo_hi = static_cast<double>(throt.speed_low_threshold);
            double lo_lo = lo_hi - zone;
            if (v <= lo_lo) {
                long_current_ = throt.current_low_speed;
            } else if (zone > 0.0 && v < lo_hi) {
                double t = (v - lo_lo) / zone;
                long_current_ = (1.0 - t) * throt.current_low_speed + t * long_current_;
            }

            double hi_lo = static_cast<double>(throt.speed_high_threshold);
            double hi_hi = hi_lo + zone;
            if (v >= hi_hi) {
                long_current_ = throt.current_high_speed;
            } else if (zone > 0.0 && v > hi_lo) {
                double t = (v - hi_lo) / zone;
                long_current_ = (1.0 - t) * long_current_ + t * throt.current_high_speed;
            }

            if (v > lo_hi && v < hi_lo && long_current_ > throt.current_clamp_max) {
                long_current_ = throt.current_clamp_max;
            }
        }
        cmd.throttle.data = static_cast<float>(static_cast<int>(long_current_));
        RCLCPP_DEBUG(node_->get_logger(), "[pure_pursuit] Throttle: %f, pedal ratio: %d", cmd.throttle.data, pedal_ratio_);
        pedal_ratio_ = static_cast<int>(cmd.throttle.data);

        if (steering_ < steer.mapping.clamp_min) {
            steering_ = steer.mapping.clamp_min;
        } else if (steering_ > steer.mapping.clamp_max) {
            steering_ = steer.mapping.clamp_max;
        }

        if (pedal_ratio_ < throt.pedal_min) {
            pedal_ratio_ = throt.pedal_min;
        } else if (pedal_ratio_ > throt.pedal_max) {
            pedal_ratio_ = throt.pedal_max;
        }
        finall_cmd = encoder_.encodeDrive(steering_, pedal_ratio_, racing_num_, racing_status_);
        pub_finall_cmd_->publish(finall_cmd);
        dropped_commands_ = 0;
        std_msgs::msg::UInt64 msg;
        msg.data = 0;
        dropped_commands_pub_->publish(msg);
    } else {
        RCLCPP_WARN(node_->get_logger(), "[pure_pursuit] No valid path information");
    }
}

double PurePursuitController::controlRate() const {
    return params_.system.control_rate;
}

double PurePursuitController::startupDelay() const {
    return params_.system.startup_delay;
}

void PurePursuitController::PublishShutdownBrake() {
    common_msgs::msg::HuatVehicleCmd cmd = encoder_.encodeBrake(racing_num_, 80);
    pub_finall_cmd_->publish(cmd);
    RCLCPP_WARN(node_->get_logger(), "[pure_pursuit] Shutdown brake published");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pure_pursuit_controller");
    PurePursuitController car(node);
    common_msgs::msg::HuatControlCommand cc;
    common_msgs::msg::HuatVehicleCmd a;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::Rate rate(car.controlRate());
    if (car.startupDelay() > 0.0) {
        rclcpp::Rate(1.0 / car.startupDelay()).sleep();
    }
    while (rclcpp::ok()) {
        executor.spin_some();
        car.ComputeControlCommand(cc, a);
        car.diag_updater_->force_update();
        rate.sleep();
    }
    car.PublishShutdownBrake();
    rclcpp::Rate(20.0).sleep();
    return 0;
}
