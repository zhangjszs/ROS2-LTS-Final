#ifndef PURE_PURSUIT_PURE_PURSUIT_PARAMS_H
#define PURE_PURSUIT_PURE_PURSUIT_PARAMS_H

#include <rclcpp/rclcpp.hpp>
#include <string>

struct PurePursuitParams {
    struct Topics {
        std::string path;
        std::string vehicle_state;
        std::string stop;
        std::string vehicle_command;
        std::string latency;
        std::string dropped_commands;
    } topics;

    struct System {
        double control_rate;
        double startup_delay;
        int racing_num;
    } system;

    struct Safety {
        double path_timeout;
        double state_timeout;
        // issue #12：输入来源年龄验证（独立于接收活性）。<0 = 禁用（默认，设备时钟未标定时不臆判）；
        // ≥0 = HuatCarstate header.stamp 年龄超过该容差或为异常未来时间时拒收，由接收看门狗触发 braking
        double state_source_age_tolerance_sec;
        // issue #12：回放识别改为显式配置。>0 = stamp 间隔超过该值视为回放（仅诊断不计延迟）；
        // <0 = 严格模式，取消 10s 猜测，一律按真实延迟判定
        double path_replay_stamp_gap_sec;
    } safety;

    struct Algorithm {
        struct Steering {
            double delta_max;
            double lookahead_base;
            double lookahead_speed_gain;
            double lookahead_curvature_gain;
            double lookahead_min;
            double pure_pursuit_gain;
            double filter_threshold;
            double filter_blend_ratio;
            struct Mapping {
                // 见 common_msgs/steering_calibration.h：raw = neutral + deg * units_per_degree，clamp 到 [min_raw,
                // max_raw]
                double units_per_degree;
                int neutral;
                int min_raw;
                int max_raw;
            } mapping;
        } steering;

        struct Throttle {
            double target_speed;
            double pid_kp;
            double pid_ki;
            double pid_integral_max;
            double speed_low_threshold;
            int current_low_speed;
            double speed_high_threshold;
            int current_high_speed;
            int current_clamp_max;
            int pedal_min;
            int pedal_max;
            double speed_blend_zone;  // 速度边界过渡带宽度（m/s），0 = 不过渡
        } throttle;

        struct PathSearch {
            // 从上一帧目标点向前搜索的最大索引数；首帧全量扫描
            int search_window;
            int search_window_max;    // 大曲率时扩展的上限
            int backtrack_window;     // 允许向后回看的索引数，吸收定位跳变
            int end_decel_points;     // 距路径末尾多少点时触发减速停车
            double max_crosstrack_m;  // 最近点距离超过此值则刹车，不跟飞点
        } path_search;
    } algorithm;

    explicit PurePursuitParams(rclcpp::Node::SharedPtr node);
};

#endif
