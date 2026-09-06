#include "pure_pursuit/pure_pursuit_params.h"

#include <rclcpp/rclcpp.hpp>

PurePursuitParams::PurePursuitParams(rclcpp::Node::SharedPtr node) {
    auto get = [&]<typename T>(const std::string& name, const T& default_val) -> T {
        T val;
        node->get_parameter_or<T>(name, val, default_val);
        return val;
    };

    // 话题配置 (C++20 指定初始化器)
    topics = Topics{
        .path = get("topics.path", std::string("/planning/pathlimits")),
        .vehicle_state = get("topics.vehicle_state", std::string("/localization/vehicle_state")),
        .stop = get("topics.stop", std::string("/system/stop")),
        .vehicle_command = get("topics.vehicle_command", std::string("/control/vehicle_command")),
        .latency = get("topics.latency", std::string("/debug/pure_pursuit/latency")),
        .dropped_commands = get("topics.dropped_commands", std::string("/debug/pure_pursuit/dropped_commands"))
    };

    // 系统配置 (C++20 指定初始化器)
    double control_rate = get("system.control_rate", 50.0);
    if (control_rate < 1.0) {
        RCLCPP_WARN(node->get_logger(), "[pure_pursuit] Invalid control_rate %.2f, clamping to 1 Hz", control_rate);
        control_rate = 1.0;
    }
    system = System{
        .control_rate = control_rate,
        .startup_delay = get("system.startup_delay", 0.0),
        .racing_num = get("system.racing_num", 1)
    };

    // 安全配置 (C++20 指定初始化器)
    safety = Safety{
        .path_timeout = get("safety.path_timeout", 0.5),
        .state_timeout = get("safety.state_timeout", 0.3)
    };

    // 算法配置 (C++20 嵌套指定初始化器)
    double lookahead_min = get("algorithm.steering.lookahead_min", 1.0);
    if (lookahead_min < 0.1) {
        RCLCPP_WARN(node->get_logger(), "[pure_pursuit] lookahead_min %.2f too small, clamping to 0.1", lookahead_min);
        lookahead_min = 0.1;
    }

    algorithm = Algorithm{
        .steering = {
            .delta_max = get("algorithm.steering.delta_max", 0.4),
            .lookahead_base = get("algorithm.steering.lookahead_base", 2.1),
            .lookahead_speed_gain = get("algorithm.steering.lookahead_speed_gain", 0.15),
            .lookahead_curvature_gain = get("algorithm.steering.lookahead_curvature_gain", 0.5),
            .lookahead_min = lookahead_min,
            .pure_pursuit_gain = get("algorithm.steering.pure_pursuit_gain", 3.1),
            .filter_threshold = get("algorithm.steering.filter_threshold", 0.2),
            .filter_blend_ratio = get("algorithm.steering.filter_blend_ratio", 0.8),
            .mapping = {
                .deg_per_rad = get("algorithm.steering.mapping.deg_per_rad", 3.73),
                .center_offset = get("algorithm.steering.mapping.center_offset", 110),
                .clamp_min = get("algorithm.steering.mapping.clamp_min", 0),
                .clamp_max = get("algorithm.steering.mapping.clamp_max", 220)
            }
        },
        .throttle = {
            .target_speed = get("algorithm.throttle.target_speed", 2.0),
            .pid_kp = get("algorithm.throttle.pid_kp", 0.5),
            .pid_ki = get("algorithm.throttle.pid_ki", 0.1),
            .pid_integral_max = get("algorithm.throttle.pid_integral_max", 20.0),
            .speed_low_threshold = get("algorithm.throttle.speed_low_threshold", 0.5),
            .current_low_speed = get("algorithm.throttle.current_low_speed", 30),
            .speed_high_threshold = get("algorithm.throttle.speed_high_threshold", 2.0),
            .current_high_speed = get("algorithm.throttle.current_high_speed", 15),
            .current_clamp_max = get("algorithm.throttle.current_clamp_max", 30),
            .pedal_min = get("algorithm.throttle.pedal_min", 0),
            .pedal_max = get("algorithm.throttle.pedal_max", 100),
            .speed_blend_zone = get("algorithm.throttle.speed_blend_zone", 0.3)
        },
        .path_search = {
            .search_window = get("algorithm.path_search.search_window", 50),
            .search_window_max = get("algorithm.path_search.search_window_max", 150),
            .backtrack_window = get("algorithm.path_search.backtrack_window", 15),
            .end_decel_points = get("algorithm.path_search.end_decel_points", 10),
            .max_crosstrack_m = get("algorithm.path_search.max_crosstrack_m", 6.0)
        }
    };
}
