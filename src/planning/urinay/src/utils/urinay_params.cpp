/**
 * @file Param.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief UrinayParams 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "utils/urinay_params.hpp"

/* ----------------------------- 私有方法 ---------------------------- */

/* ----------------------------- 公有方法 ----------------------------- */

UrinayParams::UrinayParams(rclcpp::Node *const nh) {
    auto declare_and_get = [&]<typename T>(const std::string& name, const T& default_val) -> T {
        if (!nh->has_parameter(name)) {
            nh->declare_parameter<T>(name, default_val);
        }
        T val{};
        nh->get_parameter(name, val);
        return val;
    };

    // 主参数 (C++20 指定初始化器，严格按成员声明顺序)
    main = Main{
        .input_cones_topic = declare_and_get("input_cones_topic", std::string("/sensors/cones/fused")),
        .input_pose_topic = declare_and_get("input_pose_topic", std::string("/localization/vehicle_state")),
        .output_topic = declare_and_get("output_topic", std::string("/planning/track/pathlimits")),
        .stop_topic = declare_and_get("stop_topic", std::string("/planning/track/stop_request")),
        .markers_full_topic = {},
        .markers_partial_topic = {},
        .profiling_topic = declare_and_get("profiling_topic", std::string("/debug/urinay/profiling")),
        .package_path = ament_index_cpp::get_package_share_directory("urinay"),
        .shutdown_on_loop_closure = declare_and_get("shutdown_on_loop_closure", true),
        .enable_profiling = declare_and_get("enable_profiling", true),
        .min_cone_confidence = static_cast<float>(declare_and_get("min_cone_confidence", 0.0)),
        .number_of_stopped_turns = declare_and_get("number_of_stopped_turns", 3),
        .the_mode_of_path = declare_and_get("the_mode_of_path", 4),
        .start_x = declare_and_get("start_x", 0.0),
        .start_y = declare_and_get("start_y", 0.0)
    };

    // WayComputer 参数 (C++20 嵌套指定初始化器)
    wayComputer = WayComputer{
        .max_triangle_edge_len = declare_and_get("max_triangle_edge_len", 9.0),
        .min_triangle_angle = declare_and_get("min_triangle_angle", 0.25),
        .max_dist_circum_midPoint = declare_and_get("max_dist_circum_midPoint", 1.0),
        .failsafe_max_way_horizon_size = declare_and_get("failsafe_max_way_horizon_size", 6),
        .general_failsafe = declare_and_get("general_failsafe", true),
        .general_failsafe_safetyFactor = declare_and_get("general_failsafe_safetyFactor", 1.4),
        .search = {
            .max_way_horizon_size = declare_and_get("max_way_horizon_size", 0),
            .max_search_tree_height = declare_and_get("max_search_tree_height", 5),
            .search_radius = declare_and_get("search_radius", 5.0),
            .max_angle_diff = declare_and_get("max_angle_diff", 0.6),
            .edge_len_diff_factor = declare_and_get("edge_len_diff_factor", 0.5),
            .max_search_options = declare_and_get("max_search_options", 2),
            .max_next_heuristic = declare_and_get("max_next_heuristic", 3.0),
            .heur_dist_ponderation = static_cast<float>(declare_and_get("heur_dist_ponderation", 0.6)),
            .allow_intersection = declare_and_get("allow_intersection", false),
            .max_treeSearch_time = static_cast<float>(declare_and_get("max_treeSearch_time", 0.05))
        },
        .way = {
            .max_dist_loop_closure = declare_and_get("max_dist_loop_closure", 1.0),
            .max_angle_diff_loop_closure = declare_and_get("max_angle_diff_loop_closure", 0.6),
            .vital_num_midpoints = declare_and_get("vital_num_midpoints", 5)
        }
    };

    // 可视化参数 (C++20 指定初始化器)
    visualization = Visualization{
        .publish_markers = declare_and_get("publish_markers", false),
        .triangulation_topic = declare_and_get("marker_topics.triangulation", std::string("/visualization/urinay/triangulation")),
        .midpoints_topic = declare_and_get("marker_topics.midpoints", std::string("/visualization/urinay/midpoints")),
        .way_topic = declare_and_get("marker_topics.way", std::string("/visualization/urinay/way"))
    };
}
