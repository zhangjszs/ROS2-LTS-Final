/**
 * @file urinay_params.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 UrinayParams 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <ament_index_cpp/get_package_share_path.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 表示程序所需的所有参数，按模块和结构划分。
 * 该类旨在作为 .yaml 文件与程序模块
 * 和结构之间的桥梁。
 * 构造时，使用 ROS 参数填充所有参数。
 */
class UrinayParams {
   public:
    UrinayParams(rclcpp::Node *const nh);
    struct Main {
        std::string input_cones_topic, input_pose_topic, output_topic, stop_topic;
        std::string markers_full_topic, markers_partial_topic;
        std::string profiling_topic;
        std::string package_path;
        bool shutdown_on_loop_closure;
        bool enable_profiling;
        float min_cone_confidence;
        int number_of_stopped_turns;
        int the_mode_of_path;
        double start_x;
        double start_y;
    } main;
    struct WayComputer {
        double max_triangle_edge_len, min_triangle_angle, max_dist_circum_midPoint;
        int failsafe_max_way_horizon_size;
        bool general_failsafe;
        double general_failsafe_safetyFactor;
        struct Search {
            int max_way_horizon_size;
            int max_search_tree_height;
            double search_radius, max_angle_diff, edge_len_diff_factor;
            int max_search_options;
            double max_next_heuristic;
            float heur_dist_ponderation;
            bool allow_intersection;
            float max_treeSearch_time;
        } search;
        struct Way {
            double max_dist_loop_closure;
            double max_angle_diff_loop_closure;
            int vital_num_midpoints;
        } way;
    } wayComputer;
    struct Visualization {
        bool publish_markers;
        std::string triangulation_topic;
        std::string midpoints_topic;
        std::string way_topic;
    } visualization;
};
