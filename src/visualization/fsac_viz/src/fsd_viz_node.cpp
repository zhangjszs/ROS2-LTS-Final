#include <array>
#include <cmath>
#include <deque>
#include <format>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common_msgs/msg/huat_asensing.hpp"
#include "common_msgs/msg/huat_carstate.hpp"
#include "common_msgs/msg/huat_cone.hpp"
#include "common_msgs/msg/huat_cone_cluster.hpp"
#include "common_msgs/msg/huat_map.hpp"
#include "common_msgs/msg/huat_path_limits.hpp"
#include "common_msgs/msg/huat_tracklimits.hpp"
#include "cone_types.h"

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kLidarToImuDistance = 1.87;
constexpr double kDegToRad = kPi / 180.0;

namespace {

// 颜色工具
struct Color {
    double r, g, b, a;
    Color(double rr = 1.0, double gg = 1.0, double bb = 1.0, double aa = 1.0) : r(rr), g(gg), b(bb), a(aa) {}
};

[[maybe_unused]] static Color ColorFromHex(uint32_t hex, double alpha = 1.0) {
    return Color(((hex >> 16) & 0xFF) / 255.0, ((hex >> 8) & 0xFF) / 255.0, (hex & 0xFF) / 255.0, alpha);
}

static std_msgs::msg::ColorRGBA ToRosColor(const Color& c) {
    std_msgs::msg::ColorRGBA rgba;
    rgba.r = c.r;
    rgba.g = c.g;
    rgba.b = c.b;
    rgba.a = c.a;
    return rgba;
}

// ------------------------------------------------------------------
// 锥桶可视化器
// ------------------------------------------------------------------
class ConeVisualizer {
   public:
    explicit ConeVisualizer(rclcpp::Node::SharedPtr node) : node_(node) {
        node_->declare_parameter("fixed_frame", "map");
        node_->declare_parameter("use_mesh", false);
        node_->declare_parameter("cone_mesh", "package://fsac_viz/meshes/construction_cone.dae");
        node_->declare_parameter("cone_scale", 0.3);
        node_->declare_parameter("min_alpha", 0.3);
        node_->declare_parameter("show_confidence_alpha", true);
        node_->declare_parameter("show_distance_label", true);
        node_->declare_parameter("show_cluster_bbox", false);
        node_->declare_parameter("marker_lifetime", 0.5);
        node_->declare_parameter("persistent_cones", true);
        node_->declare_parameter("cone_map_topic", "/sensors/cones/fused");
        node_->declare_parameter("cone_cluster_topic", "/sensors/cones/raw");
        node_->declare_parameter("cone_marker_topic", "/fsd/viz/cones");

        node_->get_parameter("fixed_frame", fixed_frame_);
        node_->get_parameter("use_mesh", use_mesh_);
        node_->get_parameter("cone_mesh", cone_mesh_);
        node_->get_parameter("cone_scale", cone_scale_);
        node_->get_parameter("min_alpha", min_alpha_);
        node_->get_parameter("show_confidence_alpha", show_confidence_alpha_);
        node_->get_parameter("show_distance_label", show_distance_label_);
        node_->get_parameter("show_cluster_bbox", show_cluster_bbox_);
        node_->get_parameter("marker_lifetime", marker_lifetime_);
        node_->get_parameter("persistent_cones", persistent_cones_);
        std::string cone_map_topic;
        std::string cone_cluster_topic;
        std::string cone_marker_topic;
        node_->get_parameter("cone_map_topic", cone_map_topic);
        node_->get_parameter("cone_cluster_topic", cone_cluster_topic);
        node_->get_parameter("cone_marker_topic", cone_marker_topic);

        // 从参数映射颜色（默认：FSSIM 约定）
        std::vector<double> c;
        node_->declare_parameter("color_blue", std::vector<double>{0.0, 0.0, 1.0, 1.0});
        node_->declare_parameter("color_yellow", std::vector<double>{1.0, 0.85, 0.0, 1.0});
        node_->declare_parameter("color_orange", std::vector<double>{1.0, 0.5, 0.0, 1.0});
        node_->declare_parameter("color_unknown", std::vector<double>{0.8, 0.8, 0.8, 1.0});

        node_->get_parameter("color_blue", c);
        color_blue_ = c.size() >= 3 ? Color(c[0], c[1], c[2], c.size() >= 4 ? c[3] : 1.0) : Color(0.0, 0.0, 1.0);
        c.clear();
        node_->get_parameter("color_yellow", c);
        color_yellow_ = c.size() >= 3 ? Color(c[0], c[1], c[2], c.size() >= 4 ? c[3] : 1.0) : Color(1.0, 0.85, 0.0);
        c.clear();
        node_->get_parameter("color_orange", c);
        color_orange_ = c.size() >= 3 ? Color(c[0], c[1], c[2], c.size() >= 4 ? c[3] : 1.0) : Color(1.0, 0.5, 0.0);
        c.clear();
        node_->get_parameter("color_unknown", c);
        color_unknown_ = c.size() >= 3 ? Color(c[0], c[1], c[2], c.size() >= 4 ? c[3] : 1.0) : Color(0.8, 0.8, 0.8);

        cone_map_sub_ = node_->create_subscription<common_msgs::msg::HuatMap>(
            cone_map_topic, 10, [this](const common_msgs::msg::HuatMap::ConstSharedPtr msg) { OnConeMap(msg); });
        cone_cluster_sub_ = node_->create_subscription<common_msgs::msg::HuatConeCluster>(
            cone_cluster_topic, 10,
            [this](const common_msgs::msg::HuatConeCluster::ConstSharedPtr msg) { OnConeCluster(msg); });
        marker_pub_ =
            node_->create_publisher<visualization_msgs::msg::MarkerArray>(cone_marker_topic, 1);  // latch=true（锁存）

        // TF 缓冲区，用于自适应坐标变换
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(node_->get_logger(), "[ConeVisualizer] fixed_frame=%s use_mesh=%d map=%s raw=%s marker=%s",
                    fixed_frame_.c_str(), use_mesh_, cone_map_topic.c_str(), cone_cluster_topic.c_str(),
                    cone_marker_topic.c_str());
    }

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatMap>::SharedPtr cone_map_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatConeCluster>::SharedPtr cone_cluster_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string fixed_frame_;
    bool use_mesh_;
    std::string cone_mesh_;
    double cone_scale_;
    double min_alpha_;
    bool show_confidence_alpha_;
    bool show_distance_label_;
    bool show_cluster_bbox_;
    bool persistent_cones_;
    double marker_lifetime_;
    Color color_blue_, color_yellow_, color_orange_, color_unknown_;

    // 缓存最新聚类颜色，以近似位置为键
    std::map<std::string, char> latest_cluster_colors_;
    rclcpp::Time last_cluster_time_;

    // persistent_cones_ 模式下追踪上一帧已发布的 ID，用于删除消失的锥桶
    std::set<int> prev_cone_ids_;

    void OnConeCluster(const common_msgs::msg::HuatConeCluster::ConstSharedPtr& msg) {
        latest_cluster_colors_.clear();
        if (msg->color.data.empty())
            return;
        for (size_t i = 0; i < msg->points.size() && i < msg->color.data.size(); ++i) {
            std::string key = PosKey(msg->points[i].x, msg->points[i].y);
            latest_cluster_colors_[key] = msg->color.data[i];
        }
        last_cluster_time_ = node_->now();
    }

    static std::string PosKey(float x, float y) {
        return std::format("{},{}", static_cast<int>(std::round(x * 10)), static_cast<int>(std::round(y * 10)));
    }

    char LookupClusterColor(float x, float y) const {
        if ((node_->now() - last_cluster_time_).seconds() > 1.0)
            return 'n';
        auto it = latest_cluster_colors_.find(PosKey(x, y));
        if (it != latest_cluster_colors_.end())
            return it->second;
        // 尝试邻近网格
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0)
                    continue;
                std::string key = std::format("{},{}", static_cast<int>(std::round(x * 10)) + dx,
                                              static_cast<int>(std::round(y * 10)) + dy);
                auto it2 = latest_cluster_colors_.find(key);
                if (it2 != latest_cluster_colors_.end())
                    return it2->second;
            }
        }
        return 'n';
    }

    Color GetConeColor(const common_msgs::msg::HuatCone& cone, float local_x, float local_y) const {
        // 优先尝试聚类颜色
        char cc = LookupClusterColor(local_x, local_y);
        if (cc == 'b')
            return color_blue_;
        if (cc == 'y')
            return color_yellow_;
        if (cc == 'r')
            return color_orange_;

        // 回退：HuatCone.type 使用视觉颜色枚举
        switch (cone.type) {
            case huat_cone::BLUE:
                return color_blue_;
            case huat_cone::YELLOW_SMALL:
                return color_yellow_;
            case huat_cone::YELLOW_BIG:
                return color_orange_;
            case huat_cone::RED:
                return color_orange_;
            default:
                return color_unknown_;
        }
    }

    static double ConfidenceToAlpha(uint32_t conf, double min_a) {
        // confidence 为 uint32，通常为 0-100
        double c = std::min(100.0, static_cast<double>(conf));
        return std::max(min_a, c / 100.0);
    }

    // Returns the frame_id that `pose` is expressed in after this call.
    std::string TransformPose(geometry_msgs::msg::Pose& pose, const std::string& from_frame,
                              const rclcpp::Time& stamp) {
        if (from_frame.empty() || from_frame == fixed_frame_)
            return fixed_frame_;
        try {
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = tf_buffer_->lookupTransform(fixed_frame_, from_frame, stamp, rclcpp::Duration::from_seconds(0.05));
            } catch (const tf2::ExtrapolationException&) {
                tf = tf_buffer_->lookupTransform(fixed_frame_, from_frame, rclcpp::Time(0, 0),
                                                 rclcpp::Duration::from_seconds(0.05));
            }
            geometry_msgs::msg::Pose pose_out;
            tf2::doTransform(pose, pose_out, tf);
            pose = pose_out;
            return fixed_frame_;
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "[ConeVisualizer] TF fail %s->%s: %s (markers stay in %s)", from_frame.c_str(),
                                 fixed_frame_.c_str(), e.what(), from_frame.c_str());
            return from_frame;
        }
    }

    void OnConeMap(const common_msgs::msg::HuatMap::ConstSharedPtr& msg) {
        visualization_msgs::msg::MarkerArray array;
        rclcpp::Time now = node_->now();
        int id = 0;
        int fallback_id = std::numeric_limits<int>::max() / 2;

        for (const auto& cone : msg->cone) {
            const int marker_id = persistent_cones_ ? GetStableMarkerId(cone, fallback_id) : id++;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = fixed_frame_;
            marker.header.stamp = now;
            marker.ns = "cones";
            marker.id = marker_id;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime =
                persistent_cones_ ? rclcpp::Duration(0, 0) : rclcpp::Duration::from_seconds(marker_lifetime_);

            if (use_mesh_) {
                marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
                marker.mesh_resource = cone_mesh_;
                marker.mesh_use_embedded_materials = false;
            } else {
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
            }

            geometry_msgs::msg::Pose pose;
            pose.position.x = cone.position_global.x;
            pose.position.y = cone.position_global.y;
            pose.position.z = cone.position_global.z;
            pose.orientation.w = 1.0;

            const std::string src_frame = msg->header.frame_id.empty() ? fixed_frame_ : msg->header.frame_id;
            marker.header.frame_id = TransformPose(pose, src_frame, msg->header.stamp);

            marker.pose = pose;
            marker.scale.x = cone_scale_;
            marker.scale.y = cone_scale_;
            marker.scale.z = cone_scale_ * 1.5;

            Color c = GetConeColor(cone, cone.position_base_link.x, cone.position_base_link.y);
            double alpha = show_confidence_alpha_ ? ConfidenceToAlpha(cone.confidence, min_alpha_) : 1.0;
            marker.color = ToRosColor(Color(c.r, c.g, c.b, alpha));

            array.markers.push_back(marker);

            // 距离标签
            if (show_distance_label_) {
                visualization_msgs::msg::Marker text;
                text.header = marker.header;
                text.ns = "cone_labels";
                text.id = marker_id;
                text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text.action = visualization_msgs::msg::Marker::ADD;
                text.lifetime = marker.lifetime;
                text.pose.position = pose.position;
                text.pose.position.z += 0.5;
                text.scale.z = 0.25;
                float dist = std::sqrt(cone.position_base_link.x * cone.position_base_link.x +
                                       cone.position_base_link.y * cone.position_base_link.y);
                text.text = std::format("{:.1f}m", dist);
                text.color = ToRosColor(Color(1.0, 1.0, 1.0, 0.8));
                array.markers.push_back(text);
            }
        }

        if (persistent_cones_) {
            // 删除上一帧有、本帧没有的锥桶（避免永久残影）
            std::set<int> cur_ids;
            for (const auto& m : array.markers) {
                if (m.ns == "cones")
                    cur_ids.insert(m.id);
            }
            for (int gone_id : prev_cone_ids_) {
                if (cur_ids.count(gone_id))
                    continue;
                for (const char* ns : {"cones", "cone_labels"}) {
                    visualization_msgs::msg::Marker del;
                    del.header.frame_id = fixed_frame_;
                    del.header.stamp = now;
                    del.ns = ns;
                    del.id = gone_id;
                    del.action = visualization_msgs::msg::Marker::DELETE;
                    array.markers.push_back(del);
                }
            }
            prev_cone_ids_ = cur_ids;
            marker_pub_->publish(array);
            return;
        }

        // 如果数量减少，通过发布更高 ID 的空标记来删除旧标记
        static int last_id_count = 0;
        for (int i = id; i < last_id_count; ++i) {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = fixed_frame_;
            del.header.stamp = now;
            del.ns = "cones";
            del.id = i;
            del.action = visualization_msgs::msg::Marker::DELETE;
            array.markers.push_back(del);

            visualization_msgs::msg::Marker del_text;
            del_text.header = del.header;
            del_text.ns = "cone_labels";
            del_text.id = i;
            del_text.action = visualization_msgs::msg::Marker::DELETE;
            array.markers.push_back(del_text);
        }
        last_id_count = id;

        marker_pub_->publish(array);
    }

    static int GetStableMarkerId(const common_msgs::msg::HuatCone& cone, int& fallback_id) {
        if (cone.id <= static_cast<uint32_t>(std::numeric_limits<int>::max())) {
            return static_cast<int>(cone.id);
        }
        return fallback_id++;
    }
};

// ------------------------------------------------------------------
// 车辆可视化器
// ------------------------------------------------------------------
class VehicleVisualizer {
   public:
    explicit VehicleVisualizer(rclcpp::Node::SharedPtr node) : node_(node) {
        node_->declare_parameter("fixed_frame", "map");
        node_->declare_parameter("use_mesh", false);
        node_->declare_parameter("car_mesh", "package://fsac_viz/meshes/whole_car.stl");
        node_->declare_parameter("car_scale", 0.001);
        node_->declare_parameter("show_trail", true);
        node_->declare_parameter("trail_max_points", 200);
        node_->declare_parameter("trail_min_dist", 0.1);
        node_->declare_parameter("show_velocity_arrow", true);
        node_->declare_parameter("show_status_text", true);
        node_->declare_parameter("show_heading_arrow", true);
        node_->declare_parameter("marker_lifetime", 0.3);
        node_->declare_parameter("publish_rate", 30.0);
        node_->declare_parameter("vehicle_state_topic", "/localization/vehicle_state");
        node_->declare_parameter("ins_topic", "/INS/ASENSING_INS");
        node_->declare_parameter("vehicle_marker_topic", "/fsd/viz/vehicle");

        node_->get_parameter("fixed_frame", fixed_frame_);
        node_->get_parameter("use_mesh", use_mesh_);
        node_->get_parameter("car_mesh", car_mesh_);
        node_->get_parameter("car_scale", car_scale_);
        node_->get_parameter("show_trail", show_trail_);
        node_->get_parameter("trail_max_points", trail_max_points_);
        node_->get_parameter("trail_min_dist", trail_min_dist_);
        node_->get_parameter("show_velocity_arrow", show_velocity_arrow_);
        node_->get_parameter("show_status_text", show_status_text_);
        node_->get_parameter("show_heading_arrow", show_heading_arrow_);
        node_->get_parameter("marker_lifetime", marker_lifetime_);
        node_->get_parameter("publish_rate", publish_rate_);
        std::string vehicle_state_topic;
        std::string ins_topic;
        std::string vehicle_marker_topic;
        node_->get_parameter("vehicle_state_topic", vehicle_state_topic);
        node_->get_parameter("ins_topic", ins_topic);
        node_->get_parameter("vehicle_marker_topic", vehicle_marker_topic);

        state_sub_ = node_->create_subscription<common_msgs::msg::HuatCarstate>(
            vehicle_state_topic, 10,
            [this](const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnStateMessage(msg); });
        ins_sub_ = node_->create_subscription<common_msgs::msg::HuatASENSING>(
            ins_topic, 10, [this](const common_msgs::msg::HuatASENSING::ConstSharedPtr msg) { OnInsMessage(msg); });
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(vehicle_marker_topic, 1);

        // 定时器驱动发布：即使状态消息速率低或抖动也能平滑显示
        timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_), [this] { OnTimer(); });

        // 预计算车轮偏移量（以后轴为中心）
        wheel_offsets_[0] = {0.0, 0.35, 0.0};
        wheel_offsets_[1] = {0.0, -0.35, 0.0};
        wheel_offsets_[2] = {-1.2, 0.35, 0.0};
        wheel_offsets_[3] = {-1.2, -0.35, 0.0};

        RCLCPP_INFO(node_->get_logger(),
                    "[VehicleVisualizer] fixed_frame=%s state=%s ins=%s marker=%s publish_rate=%.1fHz lifetime=%.2fs",
                    fixed_frame_.c_str(), vehicle_state_topic.c_str(), ins_topic.c_str(), vehicle_marker_topic.c_str(),
                    publish_rate_, marker_lifetime_);
    }

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatCarstate>::SharedPtr state_sub_;
    rclcpp::Subscription<common_msgs::msg::HuatASENSING>::SharedPtr ins_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string fixed_frame_;
    bool use_mesh_;
    std::string car_mesh_;
    double car_scale_;
    bool show_trail_;
    int trail_max_points_;
    double trail_min_dist_;
    bool show_velocity_arrow_;
    bool show_status_text_;
    bool show_heading_arrow_;
    double marker_lifetime_;
    double publish_rate_;

    common_msgs::msg::HuatCarstate last_state_;
    bool has_state_ = false;
    double ins_roll_ = 0.0, ins_pitch_ = 0.0;

    struct TrailPoint {
        double x, y, z;
    };
    std::deque<TrailPoint> trail_;

    // 预计算的车轮偏移量
    std::array<std::array<double, 3>, 4> wheel_offsets_;

    void OnInsMessage(const common_msgs::msg::HuatASENSING::ConstSharedPtr& msg) {
        ins_roll_ = msg->roll;
        ins_pitch_ = msg->pitch;
    }

    void OnStateMessage(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_state_ = *msg;
        has_state_ = true;

        double cx = msg->car_state.x;
        double cy = msg->car_state.y;

        // 如果移动距离足够则追加轨迹
        if (show_trail_) {
            if (trail_.empty() || std::hypot(trail_.back().x - cx, trail_.back().y - cy) > trail_min_dist_) {
                trail_.push_back({cx, cy, 0.0});
                if (static_cast<int>(trail_.size()) > trail_max_points_)
                    trail_.pop_front();
            }
        }
    }

    void OnTimer() { PublishVehicleMarkers(); }

    std::mutex mutex_;

    void PublishVehicleMarkers() {
        common_msgs::msg::HuatCarstate state;
        bool has_state;
        std::deque<TrailPoint> trail_copy;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            state = last_state_;
            has_state = has_state_;
            trail_copy = trail_;
        }

        if (!has_state)
            return;

        visualization_msgs::msg::MarkerArray array;
        rclcpp::Time now = node_->now();
        const auto& s = state.car_state;
        const double v = state.v;

        double cx = s.x;
        double cy = s.y;
        double cz = 0.0;
        double yaw = s.theta;
        double c = std::cos(yaw);
        double sn = std::sin(yaw);

        rclcpp::Duration life = rclcpp::Duration::from_seconds(marker_lifetime_);

        // 1. 车身（立方体或网格模型）
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = fixed_frame_;
            m.header.stamp = now;
            m.ns = "vehicle_body";
            m.id = 0;
            m.lifetime = life;
            if (use_mesh_) {
                m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
                m.mesh_resource = car_mesh_;
                m.mesh_use_embedded_materials = false;
                m.pose.position.x = cx;
                m.pose.position.y = cy;
                m.pose.position.z = cz;
                m.pose.orientation.w = std::cos(yaw * 0.5);
                m.pose.orientation.z = std::sin(yaw * 0.5);
                m.scale.x = m.scale.y = m.scale.z = car_scale_;
                m.color = ToRosColor(Color(0.75, 0.75, 0.75));
            } else {
                m.type = visualization_msgs::msg::Marker::CUBE;
                m.pose.position.x = cx - 0.75 * c;
                m.pose.position.y = cy - 0.75 * sn;
                m.pose.position.z = cz + 0.15;
                m.pose.orientation.w = std::cos(yaw * 0.5);
                m.pose.orientation.z = std::sin(yaw * 0.5);
                m.scale.x = 1.5;
                m.scale.y = 0.5;
                m.scale.z = 0.3;
                m.color = ToRosColor(Color(0.6, 0.6, 0.6));
            }
            array.markers.push_back(m);
        }

        // 2. 航向箭头
        if (show_heading_arrow_) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = fixed_frame_;
            m.header.stamp = now;
            m.ns = "vehicle_heading";
            m.id = 1;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = life;
            geometry_msgs::msg::Point p0, p1;
            p0.x = cx;
            p0.y = cy;
            p0.z = cz + 0.4;
            p1.x = cx + 1.5 * c;
            p1.y = cy + 1.5 * sn;
            p1.z = cz + 0.4;
            m.points.push_back(p0);
            m.points.push_back(p1);
            m.scale.x = 0.15;
            m.scale.y = 0.25;
            m.scale.z = 0.2;
            m.color = ToRosColor(Color(0.2, 0.8, 0.2));
            array.markers.push_back(m);
        }

        // 3. 车轮：使用 SPHERE_LIST（一个标记，4 个球体）代替 4 个圆柱体
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = fixed_frame_;
            m.header.stamp = now;
            m.ns = "vehicle_wheels";
            m.id = 2;
            m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = life;
            m.scale.x = m.scale.y = m.scale.z = 0.22;
            m.color = ToRosColor(Color(0.1, 0.1, 0.1));

            for (const auto& off : wheel_offsets_) {
                geometry_msgs::msg::Point p;
                // 手动二维旋转 + 平移（避免 tf2::Transform 开销）
                p.x = cx + off[0] * c - off[1] * sn;
                p.y = cy + off[0] * sn + off[1] * c;
                p.z = cz + off[2] + 0.15;
                m.points.push_back(p);
            }
            array.markers.push_back(m);
        }

        // 4. 带渐隐效果的轨迹（逐顶点颜色）
        if (show_trail_ && trail_copy.size() >= 2) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = fixed_frame_;
            m.header.stamp = now;
            m.ns = "vehicle_trail";
            m.id = 3;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = life;
            m.scale.x = 0.08;

            const size_t n = trail_copy.size();
            for (size_t i = 0; i < n; ++i) {
                geometry_msgs::msg::Point p;
                p.x = trail_copy[i].x;
                p.y = trail_copy[i].y;
                p.z = trail_copy[i].z + 0.02;
                m.points.push_back(p);
                double t = static_cast<double>(i) / static_cast<double>(n);
                std_msgs::msg::ColorRGBA col;
                col.r = 0.0;
                col.g = 0.8;
                col.b = 0.8;
                col.a = t * 0.8;
                m.colors.push_back(col);
            }
            array.markers.push_back(m);
        }

        // 5. 速度箭头
        if (show_velocity_arrow_) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = fixed_frame_;
            m.header.stamp = now;
            m.ns = "vehicle_velocity";
            m.id = 4;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = life;
            geometry_msgs::msg::Point p0, p1;
            p0.x = cx;
            p0.y = cy;
            p0.z = cz + 0.5;
            p1.x = cx + v * c * 0.5;
            p1.y = cy + v * sn * 0.5;
            p1.z = cz + 0.5;
            m.points.push_back(p0);
            m.points.push_back(p1);
            m.scale.x = 0.08;
            m.scale.y = 0.15;
            m.scale.z = 0.12;
            double ratio = std::min(1.0, v / 15.0);
            m.color = ToRosColor(Color(1.0 - (1.0 - ratio) * 0.5, 1.0 - ratio * 0.8, 0.0));
            array.markers.push_back(m);
        }

        // 6. 状态文本
        if (show_status_text_) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = fixed_frame_;
            m.header.stamp = now;
            m.ns = "vehicle_status";
            m.id = 5;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = life;
            m.pose.position.x = cx;
            m.pose.position.y = cy;
            m.pose.position.z = cz + 1.2;
            m.scale.z = 0.3;
            m.text = std::format("V:{:.1f}km/h Yaw:{:.1f}deg", v * 3.6, yaw * 180.0 / kPi);
            m.color = ToRosColor(Color(1.0, 1.0, 1.0, 0.9));
            array.markers.push_back(m);
        }

        marker_pub_->publish(array);
    }
};

// ------------------------------------------------------------------
// 路径可视化器
// ------------------------------------------------------------------
class PathVisualizer {
   public:
    explicit PathVisualizer(rclcpp::Node::SharedPtr node) : node_(node) {
        node_->declare_parameter("fixed_frame", "map");
        node_->declare_parameter("show_center_path", true);
        node_->declare_parameter("show_boundaries", true);
        node_->declare_parameter("show_path_points", true);
        node_->declare_parameter("marker_lifetime", 0.5);
        node_->declare_parameter("path_topic", "/planning/pathlimits");
        node_->declare_parameter("path_marker_topic", "/fsd/viz/path");

        node_->get_parameter("fixed_frame", fixed_frame_);
        node_->get_parameter("show_center_path", show_center_path_);
        node_->get_parameter("show_boundaries", show_boundaries_);
        node_->get_parameter("show_path_points", show_path_points_);
        node_->get_parameter("marker_lifetime", marker_lifetime_);
        std::string path_topic;
        std::string path_marker_topic;
        node_->get_parameter("path_topic", path_topic);
        node_->get_parameter("path_marker_topic", path_marker_topic);

        path_sub_ = node_->create_subscription<common_msgs::msg::HuatPathLimits>(
            path_topic, 10, [this](const common_msgs::msg::HuatPathLimits::ConstSharedPtr msg) { OnPathLimits(msg); });
        path_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(path_marker_topic, 1);

        RCLCPP_INFO(node_->get_logger(), "[PathVisualizer] fixed_frame=%s path=%s marker=%s", fixed_frame_.c_str(),
                    path_topic.c_str(), path_marker_topic.c_str());
    }

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<common_msgs::msg::HuatPathLimits>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;

    std::string fixed_frame_;
    bool show_center_path_;
    bool show_boundaries_;
    bool show_path_points_;
    double marker_lifetime_;

    void OnPathLimits(const common_msgs::msg::HuatPathLimits::ConstSharedPtr& msg) {
        visualization_msgs::msg::MarkerArray array;
        rclcpp::Time now = node_->now();
        int id = 0;

        // 消息自带帧（base_link 或 map），用它而非 fixed_frame_ 避免坐标系错位
        const std::string path_frame = msg->header.frame_id.empty() ? fixed_frame_ : msg->header.frame_id;
        // tracklimits 坐标字段选择：map 帧用 position_global，否则用 position_base_link
        const bool use_global_pos = (path_frame == "map");

        // 1. 中心路径
        if (show_center_path_ && !msg->path.empty()) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = path_frame;
            m.header.stamp = now;
            m.ns = "path_center";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
            m.scale.x = 0.06;
            m.color = ToRosColor(Color(0.6, 0.2, 0.9));  // 紫色

            for (const auto& pt : msg->path) {
                geometry_msgs::msg::Point p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = 0.05;
                m.points.push_back(p);
            }
            array.markers.push_back(m);

            // 路径点（球体）
            if (show_path_points_) {
                visualization_msgs::msg::Marker sp;
                sp.header.frame_id = path_frame;
                sp.header.stamp = now;
                sp.ns = "path_points";
                sp.id = id++;
                sp.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                sp.action = visualization_msgs::msg::Marker::ADD;
                sp.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
                sp.scale.x = 0.12;
                sp.scale.y = 0.12;
                sp.scale.z = 0.12;
                sp.color = ToRosColor(Color(0.7, 0.3, 0.95, 0.8));
                for (const auto& pt : msg->path) {
                    geometry_msgs::msg::Point p;
                    p.x = pt.x;
                    p.y = pt.y;
                    p.z = 0.05;
                    sp.points.push_back(p);
                }
                array.markers.push_back(sp);
            }
        }

        // 2. 左边界（黄色）
        if (show_boundaries_ && !msg->tracklimits.left.empty()) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = path_frame;
            m.header.stamp = now;
            m.ns = "boundary_left";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
            m.scale.x = 0.04;
            m.color = ToRosColor(Color(1.0, 0.85, 0.0, 0.9));

            for (const auto& c : msg->tracklimits.left) {
                geometry_msgs::msg::Point p;
                p.x = use_global_pos ? c.position_global.x : c.position_base_link.x;
                p.y = use_global_pos ? c.position_global.y : c.position_base_link.y;
                p.z = 0.02;
                m.points.push_back(p);
            }
            array.markers.push_back(m);
        }

        // 3. 右边界（蓝色）
        if (show_boundaries_ && !msg->tracklimits.right.empty()) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = path_frame;
            m.header.stamp = now;
            m.ns = "boundary_right";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
            m.scale.x = 0.04;
            m.color = ToRosColor(Color(0.0, 0.0, 1.0, 0.9));

            for (const auto& c : msg->tracklimits.right) {
                geometry_msgs::msg::Point p;
                p.x = use_global_pos ? c.position_global.x : c.position_base_link.x;
                p.y = use_global_pos ? c.position_global.y : c.position_base_link.y;
                p.z = 0.02;
                m.points.push_back(p);
            }
            array.markers.push_back(m);
        }

        // 4. 重新规划指示器
        if (msg->replan) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = path_frame;
            m.header.stamp = now;
            m.ns = "path_replan";
            m.id = id;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
            if (!msg->path.empty()) {
                m.pose.position.x = msg->path[0].x;
                m.pose.position.y = msg->path[0].y;
                m.pose.position.z = 1.0;
            }
            m.scale.z = 0.4;
            m.text = "REPLAN";
            m.color = ToRosColor(Color(1.0, 0.0, 0.0, 1.0));
            array.markers.push_back(m);
        }

        path_pub_->publish(array);
    }
};

}  // namespace

// ------------------------------------------------------------------
// 主函数
// ------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fsd_viz_node");

    RCLCPP_INFO(node->get_logger(), "Starting fsd_viz_node...");

    ConeVisualizer cone_viz(node);
    VehicleVisualizer vehicle_viz(node);
    PathVisualizer path_viz(node);

    rclcpp::spin(node);
    return 0;
}
