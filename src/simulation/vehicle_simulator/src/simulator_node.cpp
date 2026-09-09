#include "vehicle_simulator/simulator_node.hpp"

#include <chrono>
#include <numbers>

namespace simulation {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kRadToDeg = 180.0 / kPi;

}  // namespace

SimulatorNode::SimulatorNode(const rclcpp::NodeOptions& options)
    : Node("vehicle_simulator", options), tf_broadcaster_(*this) {
    LoadParameters();
    SetupPublishersAndSubscribers();
    SetupTimers();

    if (!track_file_.empty()) {
        if (sensor_sim_.LoadTrackFromCSV(track_file_)) {
            RCLCPP_INFO(get_logger(), "Successfully loaded track with %zu cones: %s", sensor_sim_.cone_count(),
                        track_file_.c_str());
            // 发布一次全量真值地图供全局参考
            global_map_pub_->publish(sensor_sim_.GetGlobalGroundTruthMap());
        } else {
            RCLCPP_WARN(get_logger(), "Failed to load track CSV from: %s", track_file_.c_str());
        }
    }

    RCLCPP_INFO(get_logger(),
                "Vehicle simulator initialized at physics %.1fHz, sensor %.1fHz. Initial pose: (%.2f, %.2f, %.2f)",
                sim_rate_, sensor_rate_, bicycle_model_.state().x, bicycle_model_.state().y,
                bicycle_model_.state().theta);
}

void SimulatorNode::LoadParameters() {
    declare_parameter<double>("sim_rate", 100.0);
    declare_parameter<double>("sensor_rate", 20.0);
    declare_parameter<double>("fov_deg", 120.0);
    declare_parameter<double>("max_range", 15.0);
    declare_parameter<double>("noise_stddev", 0.01);
    declare_parameter<std::string>("track_file", "");

    declare_parameter<double>("init_x", 0.0);
    declare_parameter<double>("init_y", 0.0);
    declare_parameter<double>("init_theta", 0.0);
    declare_parameter<double>("init_v", 0.0);

    declare_parameter<double>("wheelbase", 1.55);
    declare_parameter<double>("max_steer_angle", 0.40);
    declare_parameter<double>("max_accel", 6.0);
    declare_parameter<double>("max_decel", 9.0);
    declare_parameter<double>("max_speed", 25.0);

    get_parameter("sim_rate", sim_rate_);
    get_parameter("sensor_rate", sensor_rate_);
    get_parameter("fov_deg", fov_deg_);
    get_parameter("max_range", max_range_);
    get_parameter("noise_stddev", noise_stddev_);
    get_parameter("track_file", track_file_);

    double init_x = 0.0, init_y = 0.0, init_theta = 0.0, init_v = 0.0;
    get_parameter("init_x", init_x);
    get_parameter("init_y", init_y);
    get_parameter("init_theta", init_theta);
    get_parameter("init_v", init_v);

    VehicleParams p;
    get_parameter("wheelbase", p.wheelbase);
    get_parameter("max_steer_angle", p.max_steer_angle);
    get_parameter("max_accel", p.max_accel);
    get_parameter("max_decel", p.max_decel);
    get_parameter("max_speed", p.max_speed);

    bicycle_model_.set_params(p);
    bicycle_model_.Reset(init_x, init_y, init_theta, init_v);
}

void SimulatorNode::SetupPublishersAndSubscribers() {
    vehicle_cmd_sub_ = create_subscription<common_msgs::msg::HuatVehicleCmd>(
        "/vehicle_command", 10,
        [this](const common_msgs::msg::HuatVehicleCmd::ConstSharedPtr msg) { OnVehicleCommand(msg); });

    control_cmd_sub_ = create_subscription<common_msgs::msg::HuatControlCommand>(
        "/control/pure_pursuit/control_cmd", 10,
        [this](const common_msgs::msg::HuatControlCommand::ConstSharedPtr msg) { OnControlCommand(msg); });

    stop_sub_ = create_subscription<common_msgs::msg::HuatStop>(
        "/system/stop", 10, [this](const common_msgs::msg::HuatStop::ConstSharedPtr msg) { OnStopMessage(msg); });

    carstate_pub_ = create_publisher<common_msgs::msg::HuatCarstate>("/localization/vehicle_state", 10);
    ins_pub_ = create_publisher<common_msgs::msg::HuatASENSING>("/INS/ASENSING_INS", 10);
    cone_map_pub_ = create_publisher<common_msgs::msg::HuatMap>("/sensors/cones/fused", 10);
    global_map_pub_ = create_publisher<common_msgs::msg::HuatMap>("/simulation/global_map", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/simulation/ground_truth_path", 10);
}

void SimulatorNode::SetupTimers() {
    auto physics_period = std::chrono::duration<double>(1.0 / sim_rate_);
    physics_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(physics_period),
                                       [this]() { UpdatePhysics(); });

    auto sensor_period = std::chrono::duration<double>(1.0 / sensor_rate_);
    sensor_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(sensor_period),
                                      [this]() { PublishSensorData(); });
}

void SimulatorNode::OnVehicleCommand(const common_msgs::msg::HuatVehicleCmd::ConstSharedPtr& msg) {
    if (stop_active_)
        return;

    // 解码转向: 90 为居中, 左转 < 90, 右转 > 90
    double steer_deg = static_cast<double>(msg->steering) - 90.0;
    current_cmd_.target_steering = steer_deg * (kPi / 180.0);

    // 解码油门与制动
    if (msg->brake_force > 0) {
        current_cmd_.target_accel =
            -(static_cast<double>(msg->brake_force) / 100.0) * bicycle_model_.params().max_decel;
    } else {
        current_cmd_.target_accel = (static_cast<double>(msg->pedal_ratio) / 100.0) * bicycle_model_.params().max_accel;
    }
}

void SimulatorNode::OnControlCommand(const common_msgs::msg::HuatControlCommand::ConstSharedPtr& msg) {
    if (stop_active_)
        return;
    current_cmd_.target_steering = msg->steering_angle.data;
    if (msg->throttle.data >= 0.0f) {
        current_cmd_.target_accel = static_cast<double>(msg->throttle.data) * bicycle_model_.params().max_accel;
    } else {
        current_cmd_.target_accel = static_cast<double>(msg->throttle.data) * bicycle_model_.params().max_decel;
    }
}

void SimulatorNode::OnStopMessage(const common_msgs::msg::HuatStop::ConstSharedPtr& msg) {
    stop_active_ = msg->stop;
    if (stop_active_) {
        current_cmd_.target_accel = -bicycle_model_.params().max_decel;
        current_cmd_.target_steering = 0.0;
    }
}

void SimulatorNode::UpdatePhysics() {
    const double dt = 1.0 / sim_rate_;
    const auto& state = bicycle_model_.Step(current_cmd_, dt);
    const auto now = this->now();

    // 1. 发布车辆状态 (/localization/vehicle_state)
    common_msgs::msg::HuatCarstate carstate_msg;
    carstate_msg.header.stamp = now;
    carstate_msg.header.frame_id = "map";
    carstate_msg.car_state.x = state.x;
    carstate_msg.car_state.y = state.y;
    carstate_msg.car_state.theta = state.theta;
    carstate_msg.v = static_cast<float>(state.v);
    carstate_pub_->publish(carstate_msg);

    // 2. 发布惯导状态 (/INS/ASENSING_INS, 供可视化或融合使用)
    common_msgs::msg::HuatASENSING ins_msg;
    ins_msg.roll = -state.yaw_rate * state.v * bicycle_model_.params().roll_gain * kRadToDeg;
    ins_msg.pitch = state.accel * bicycle_model_.params().pitch_gain * kRadToDeg;
    ins_msg.azimuth = state.theta * kRadToDeg;
    ins_msg.ground_velocity = state.v;
    ins_pub_->publish(ins_msg);

    // 3. 发布 TF 变换: map -> base_link
    PublishTf();

    // 4. 更新并发布实际行驶真值轨迹
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state.theta);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    ground_truth_path_.header.stamp = now;
    ground_truth_path_.header.frame_id = "map";
    ground_truth_path_.poses.push_back(pose);

    if (ground_truth_path_.poses.size() > 1000) {
        ground_truth_path_.poses.erase(ground_truth_path_.poses.begin());
    }
    path_pub_->publish(ground_truth_path_);
}

void SimulatorNode::PublishSensorData() {
    const auto now = this->now();
    const auto& state = bicycle_model_.state();

    // 动态模拟局部探测锥桶
    auto cone_map = sensor_sim_.GeneratePerceivedCones(state, fov_deg_, max_range_, noise_stddev_);
    cone_map.header.stamp = now;
    cone_map.header.frame_id = "map";

    cone_map_pub_->publish(cone_map);
}

void SimulatorNode::PublishTf() {
    const auto& state = bicycle_model_.state();
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = state.x;
    t.transform.translation.y = state.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state.theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(t);
}

}  // namespace simulation
