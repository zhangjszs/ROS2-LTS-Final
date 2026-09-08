#include "skidpad_planner_node.hpp"

#include <algorithm>
#include <common_msgs/msg/huat_tracklimits.hpp>
#include <ranges>

namespace skidpad {

SkidpadPlannerNode::SkidpadPlannerNode(rclcpp::Node::SharedPtr node)
    : node_(node),
      has_prev_path_(false),
      empty_count_(0),
      car_x_(0.0),
      car_y_(0.0),
      car_theta_(0.0),
      has_car_state_(false) {
    node_->declare_parameter<int>("road_type", 1);
    node_->get_parameter("road_type", road_type_);
    node_->declare_parameter<std::string>("input_cone_map_topic", "/sensors/cones/fused");
    node_->get_parameter("input_cone_map_topic", input_topic_);
    node_->declare_parameter<std::string>("output_pathlimits_topic", "/planning/skidpad/pathlimits");
    node_->get_parameter("output_pathlimits_topic", output_topic_);
    node_->declare_parameter<std::string>("vehicle_state_topic", "/localization/vehicle_state");
    node_->get_parameter("vehicle_state_topic", vehicle_state_topic_);
    node_->declare_parameter<int>("max_empty_messages_before_warn", 5);
    node_->get_parameter("max_empty_messages_before_warn", max_empty_messages_);

    double center_margin, k_a, k_r, d_0, path_spacing, path_lookahead;
    int icp_max_iter;
    double icp_convergence;
    node_->declare_parameter<double>("center_margin", 0.3);
    node_->get_parameter("center_margin", center_margin);
    center_margin_ = center_margin;
    node_->declare_parameter<int>("icp_max_iterations", 20);
    node_->get_parameter("icp_max_iterations", icp_max_iter);
    node_->declare_parameter<double>("icp_convergence_threshold", 0.001);
    node_->get_parameter("icp_convergence_threshold", icp_convergence);
    node_->declare_parameter<double>("k_a", 0.5);
    node_->get_parameter("k_a", k_a);
    node_->declare_parameter<double>("k_r", 2.0);
    node_->get_parameter("k_r", k_r);
    node_->declare_parameter<double>("d_0", 2.0);
    node_->get_parameter("d_0", d_0);
    node_->declare_parameter<double>("path_station_spacing", 0.5);
    node_->get_parameter("path_station_spacing", path_spacing);
    node_->declare_parameter<double>("path_lookahead_distance", 20.0);
    node_->get_parameter("path_lookahead_distance", path_lookahead);
    path_lookahead_ = path_lookahead;

    int apf_max_iter;
    double apf_step_limit;
    node_->declare_parameter<int>("apf_max_iterations", 10);
    node_->get_parameter("apf_max_iterations", apf_max_iter);
    node_->declare_parameter<double>("apf_step_limit", 0.2);
    node_->get_parameter("apf_step_limit", apf_step_limit);
    node_->declare_parameter<double>("icp_rmse_limit", 0.3);
    node_->get_parameter("icp_rmse_limit", icp_rmse_limit_);

    IcpApfConfig planner_cfg;
    planner_cfg.center_margin = center_margin;
    planner_cfg.icp_max_iter = icp_max_iter;
    planner_cfg.icp_convergence = icp_convergence;
    planner_cfg.k_a = k_a;
    planner_cfg.k_r = k_r;
    planner_cfg.d_0 = d_0;
    planner_cfg.path_spacing = path_spacing;
    planner_cfg.path_lookahead = path_lookahead;
    planner_cfg.apf_max_iter = apf_max_iter;
    planner_cfg.apf_step_limit = apf_step_limit;
    planner_ = std::make_unique<IcpApfPlanner>(planner_cfg);

    cone_map_sub_ = node_->create_subscription<common_msgs::msg::HuatMap>(
        input_topic_, 1, [this](const common_msgs::msg::HuatMap::ConstSharedPtr msg) { OnConeMapMessage(msg); });
    car_state_sub_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
        vehicle_state_topic_, 1,
        [this](const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnCarStateMessage(msg); });
    path_limits_pub_ = node_->create_publisher<common_msgs::msg::HuatPathLimits>(output_topic_, 1);

    RCLCPP_INFO(node_->get_logger(), "[skidpad_planner] Node initialized. road_type=%d", road_type_);
}

void SkidpadPlannerNode::Run() {
    rclcpp::spin(node_);
}

void SkidpadPlannerNode::OnCarStateMessage(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(car_state_mutex_);
    car_x_ = msg->car_state.x;
    car_y_ = msg->car_state.y;
    car_theta_ = msg->car_state.theta;
    has_car_state_ = true;
}

void SkidpadPlannerNode::UpdateBaseLinkCoords(std::vector<common_msgs::msg::HuatCone>& cones) const {
    double cx, cy, ct;
    {
        std::lock_guard<std::mutex> lock(car_state_mutex_);
        cx = car_x_;
        cy = car_y_;
        ct = car_theta_;
    }
    double cos_t = std::cos(-ct);
    double sin_t = std::sin(-ct);
    for (auto& c : cones) {
        double dx = c.position_global.x - cx;
        double dy = c.position_global.y - cy;
        c.position_base_link.x = static_cast<float>(cos_t * dx - sin_t * dy);
        c.position_base_link.y = static_cast<float>(sin_t * dx + cos_t * dy);
        c.position_base_link.z = c.position_global.z;
    }
}

std::vector<Point2D> SkidpadPlannerNode::GlobalPathToBaseLink(const std::vector<Point2D>& global_path) const {
    double cx, cy, ct;
    {
        std::lock_guard<std::mutex> lock(car_state_mutex_);
        cx = car_x_;
        cy = car_y_;
        ct = car_theta_;
    }
    double cos_t = std::cos(-ct);
    double sin_t = std::sin(-ct);
    std::vector<Point2D> result;
    result.reserve(global_path.size());
    for (const auto& p : global_path) {
        double dx = p.x - cx;
        double dy = p.y - cy;
        result.emplace_back(cos_t * dx - sin_t * dy, sin_t * dx + cos_t * dy);
    }
    return result;
}

std::vector<Point2D> SkidpadPlannerNode::BaseLinkPathToGlobal(const std::vector<Point2D>& bl_path) const {
    double cx, cy, ct;
    {
        std::lock_guard<std::mutex> lock(car_state_mutex_);
        cx = car_x_;
        cy = car_y_;
        ct = car_theta_;
    }
    double cos_t = std::cos(ct);
    double sin_t = std::sin(ct);
    std::vector<Point2D> result;
    result.reserve(bl_path.size());
    for (const auto& p : bl_path) {
        result.emplace_back(cos_t * p.x - sin_t * p.y + cx, sin_t * p.x + cos_t * p.y + cy);
    }
    return result;
}

geometry_msgs::msg::Point SkidpadPlannerNode::MakePoint(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

void SkidpadPlannerNode::PublishPrevPath(const std::vector<Point2D>& prev_path_bl) {
    common_msgs::msg::HuatPathLimits path_limits;
    path_limits.header.stamp = node_->now();
    path_limits.header.frame_id = "base_link";
    path_limits.path.reserve(prev_path_bl.size());
    std::ranges::transform(prev_path_bl, std::back_inserter(path_limits.path),
                           [](const Point2D& pt) { return MakePoint(pt.x, pt.y, 0.0); });
    path_limits.replan = false;
    path_limits_pub_->publish(path_limits);
}

bool SkidpadPlannerNode::IsPrevPathFrozen(const std::vector<Point2D>& prev_path_bl) const {
    if (prev_path_bl.empty())
        return false;
    const double freeze_threshold = path_lookahead_ * 0.8;
    const double freeze_threshold_sq = freeze_threshold * freeze_threshold;
    // 若没有任意一个点落在 freeze 阈值内（即全部点都在车辆前瞻外），判定路径已失效冻结
    const bool frozen = std::ranges::none_of(
        prev_path_bl, [freeze_threshold_sq](double d2) { return d2 <= freeze_threshold_sq; },
        [](const Point2D& p) { return p.x * p.x + p.y * p.y; });
    if (frozen) {
        RCLCPP_WARN(node_->get_logger(), "[skidpad_planner] Prev path frozen (all pts > %.1fm), resetting",
                    freeze_threshold);
    }
    return frozen;
}

void SkidpadPlannerNode::PublishEmptyPathLimits() {
    common_msgs::msg::HuatPathLimits msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";
    msg.replan = false;
    path_limits_pub_->publish(msg);
}

void SkidpadPlannerNode::OnConeMapMessage(const common_msgs::msg::HuatMap::ConstSharedPtr& msg) {
    if (road_type_ != 1) {
        return;
    }

    if (msg->cone.empty()) {
        ++empty_count_;
        if (empty_count_ <= max_empty_messages_ && has_prev_path_ && has_car_state_) {
            PublishPrevPath(GlobalPathToBaseLink(prev_path_global_));
        } else {
            PublishEmptyPathLimits();
        }
        if (empty_count_ == max_empty_messages_ + 1) {
            RCLCPP_WARN(node_->get_logger(), "[skidpad_planner] Empty cone map for %d consecutive messages",
                        empty_count_);
        }
        return;
    }

    empty_count_ = 0;

    if (!has_car_state_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "[skidpad_planner] No vehicle state received yet, skipping");
        return;
    }

    // 用当前车辆位姿把全局坐标转成当前 baseLink 坐标
    std::vector<common_msgs::msg::HuatCone> cones = msg->cone;
    UpdateBaseLinkCoords(cones);

    // 把存储的全局路径转换到当前 baseLink，作为 ICP 对齐的参考
    std::vector<Point2D> prev_path_bl;
    if (has_prev_path_) {
        prev_path_bl = GlobalPathToBaseLink(prev_path_global_);

        if (IsPrevPathFrozen(prev_path_bl)) {
            has_prev_path_ = false;
            prev_path_bl.clear();
        }
    }

    std::vector<Point2D> path = planner_->GeneratePath(cones, prev_path_bl, icp_rmse_limit_);
    if (path.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[skidpad_planner] Failed to generate path");
        if (has_prev_path_) {
            PublishPrevPath(prev_path_bl);
        } else {
            PublishEmptyPathLimits();
        }
        return;
    }

    common_msgs::msg::HuatPathLimits path_limits;
    path_limits.header.stamp = node_->now();
    path_limits.header.frame_id = "base_link";
    path_limits.path.reserve(path.size());
    std::ranges::transform(path, std::back_inserter(path_limits.path),
                           [](const Point2D& pt) { return MakePoint(pt.x, pt.y, 0.0); });

    std::ranges::copy_if(
        cones, std::back_inserter(path_limits.tracklimits.left), [this](float y) { return y < -center_margin_; },
        [](const common_msgs::msg::HuatCone& c) { return c.position_base_link.y; });
    std::ranges::copy_if(
        cones, std::back_inserter(path_limits.tracklimits.right), [this](float y) { return y > center_margin_; },
        [](const common_msgs::msg::HuatCone& c) { return c.position_base_link.y; });
    std::ranges::sort(path_limits.tracklimits.left, {}, [](const auto& c) { return c.position_base_link.x; });
    std::ranges::sort(path_limits.tracklimits.right, {}, [](const auto& c) { return c.position_base_link.x; });
    path_limits.replan = true;

    path_limits_pub_->publish(path_limits);

    prev_path_global_ = BaseLinkPathToGlobal(path);
    has_prev_path_ = true;
}

}  // namespace skidpad
