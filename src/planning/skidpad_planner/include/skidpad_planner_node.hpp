#pragma once

#include <cmath>
#include <common_msgs/msg/huat_carstate.hpp>
#include <common_msgs/msg/huat_map.hpp>
#include <common_msgs/msg/huat_path_limits.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "icp_apf_planner.hpp"

namespace skidpad {

class SkidpadPlannerNode {
   public:
    SkidpadPlannerNode(rclcpp::Node::SharedPtr node);
    void Run();

   private:
    void OnConeMapMessage(const common_msgs::msg::HuatMap::ConstSharedPtr& msg);
    void OnCarStateMessage(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg);
    void PublishEmptyPathLimits();
    void PublishPrevPath(const std::vector<Point2D>& prev_path_bl);
    bool IsPrevPathFrozen(const std::vector<Point2D>& prev_path_bl) const;
    static geometry_msgs::msg::Point MakePoint(double x, double y, double z);

    // 将锥桶的 position_global 转换到当前帧的 position_base_link
    void UpdateBaseLinkCoords(std::vector<common_msgs::msg::HuatCone>& cones) const;

    // 将全局坐标路径转换到当前帧的 baseLink 坐标
    std::vector<Point2D> GlobalPathToBaseLink(const std::vector<Point2D>& global_path) const;

    // 将 baseLink 坐标路径转换到全局坐标
    std::vector<Point2D> BaseLinkPathToGlobal(const std::vector<Point2D>& bl_path) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatMap>::SharedPtr cone_map_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr car_state_sub_;
    rclcpp::Publisher<common_msgs::msg::HuatPathLimits>::SharedPtr path_limits_pub_;

    std::unique_ptr<IcpApfPlanner> planner_;
    std::vector<Point2D> prev_path_global_;  // 全局坐标，避免车移动后坐标失效
    bool has_prev_path_;

    // #101：持久走廊模型（**全局坐标**）：瓣圆心 + 中线半径。
    // 存全局坐标的原因同 prev_path_global_——车在动，base_link 坐标每帧都变，
    // 跨帧保留模型必须用全局坐标，否则模型会跟着车一起「漂」。
    CorridorModel trusted_model_global_;
    bool has_trusted_model_;

    // #101（Planner 2026-10-28 裁定 A）：**首瓣先验**——任务层给的「先绕哪一瓣」。
    // 入口处两瓣锥桶镜面对称（几何上不可区分），必须由航线回答。存**起始位姿**
    // （全局坐标，第一帧 vehicle_state 记下），每帧换算到 base_link 后交给 Core；
    // 只在**尚未锁定瓣**（has_trusted_model_ == false）时生效。
    int initial_lobe_side_;  // +1 = 左，−1 = 右，0 = 无先验
    double start_x_;
    double start_y_;
    double start_theta_;
    bool has_start_pose_;

    int road_type_;
    std::string input_topic_;
    std::string output_topic_;
    std::string vehicle_state_topic_;
    int max_empty_messages_;
    int empty_count_;
    double center_margin_;
    double icp_rmse_limit_;

    double car_x_;
    double car_y_;
    double car_theta_;
    bool has_car_state_;
    mutable std::mutex car_state_mutex_;

    double path_lookahead_;
};

}  // namespace skidpad
