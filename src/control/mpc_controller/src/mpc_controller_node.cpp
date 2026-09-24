#include "mpc_controller/mpc_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "interface_contract.h"        // #14：统一接口契约（坐标系/有效性判定集中于此）
#include "interface_contract_qos.hpp"  // #14：stop 锁存 QoS 由契约单一来源构造

namespace mpc {

MpcControllerNode::MpcControllerNode(const rclcpp::NodeOptions& options) : Node("mpc_controller_node", options) {
    LoadParameters();
    mpc_model_.SetConfig(config_);
    SetupSubscribersAndPublishers();

    double dt = 1.0 / config_.system.control_rate;
    timer_ = create_wall_timer(std::chrono::duration<double>(dt), [this]() { ControlLoop(); });

    RCLCPP_INFO(get_logger(),
                "[MPC] Controller initialized: Rate=%.1fHz, Np=%zu, Nc=%zu, TargetSpeed=%.1fm/s, Wheelbase=%.2fm",
                config_.system.control_rate, config_.horizon.Np, config_.horizon.Nc, config_.horizon.target_speed,
                config_.system.wheelbase);
}

void MpcControllerNode::LoadParameters() {
    declare_parameter<double>("system.control_rate", 50.0);
    declare_parameter<double>("system.startup_delay", 0.0);
    declare_parameter<int>("system.racing_num", 1);
    declare_parameter<double>("system.wheelbase", 1.53);

    declare_parameter<int>("mpc.horizon_steps", 15);
    declare_parameter<int>("mpc.control_horizon", 10);
    declare_parameter<double>("mpc.sample_time", 0.05);
    declare_parameter<double>("mpc.target_speed", 8.0);

    declare_parameter<double>("mpc.weight_lat_error", 50.0);
    declare_parameter<double>("mpc.weight_heading_error", 25.0);
    declare_parameter<double>("mpc.weight_speed_error", 5.0);
    declare_parameter<double>("mpc.weight_steer_angle", 1.0);
    declare_parameter<double>("mpc.weight_accel", 0.5);
    declare_parameter<double>("mpc.weight_steer_rate", 8.0);
    declare_parameter<double>("mpc.weight_accel_rate", 2.0);

    declare_parameter<double>("mpc.max_steer_rad", 0.436332);
    declare_parameter<double>("mpc.max_steer_rate", 3.0);
    declare_parameter<double>("mpc.min_accel", -4.0);
    declare_parameter<double>("mpc.max_accel", 3.0);

    // 转向指令编码（issue #2）：默认与仿真器/评测一致（零位 90，1 raw/度，±25°），真实底盘协议由参数覆盖
    declare_parameter<double>("steering.neutral", 90.0);
    declare_parameter<double>("steering.units_per_degree", 1.0);
    declare_parameter<double>("steering.min_raw", 65.0);
    declare_parameter<double>("steering.max_raw", 115.0);

    // issue #12：状态来源年龄验证（<0 禁用；≥0 时超龄/异常未来时间不刷新接收看门狗）
    declare_parameter<double>("safety.state_source_age_tolerance_sec", -1.0);

    // issue #14：缺失显式 target_speeds 时的默认参考速度（绝不再从 Point.z 取速度）
    declare_parameter<double>("path.reference_speed_default", 0.0);

    declare_parameter<std::string>("topics.vehicle_state", "/localization/vehicle_state");
    declare_parameter<std::string>("topics.path", "/planning/skidpad_predict_path");
    declare_parameter<std::string>("topics.stop", "/system/stop");
    declare_parameter<std::string>("topics.vehicle_command", "/vehicle_command");
    declare_parameter<std::string>("topics.predicted_path", "/control/mpc_predicted_path");
    declare_parameter<std::string>("topics.reference_path", "/control/mpc_reference_path");

    get_parameter("system.control_rate", config_.system.control_rate);
    get_parameter("system.startup_delay", config_.system.startup_delay);
    get_parameter("system.racing_num", config_.system.racing_num);
    get_parameter("system.wheelbase", config_.system.wheelbase);

    int np = 15;
    int nc = 10;
    get_parameter("mpc.horizon_steps", np);
    get_parameter("mpc.control_horizon", nc);
    config_.horizon.Np = static_cast<size_t>(std::max(3, np));
    config_.horizon.Nc = static_cast<size_t>(std::max(2, std::min(nc, np)));
    get_parameter("mpc.sample_time", config_.horizon.Ts);
    get_parameter("mpc.target_speed", config_.horizon.target_speed);

    get_parameter("mpc.weight_lat_error", config_.weights.q_lat);
    get_parameter("mpc.weight_heading_error", config_.weights.q_heading);
    get_parameter("mpc.weight_speed_error", config_.weights.q_speed);
    get_parameter("mpc.weight_steer_angle", config_.weights.r_steer);
    get_parameter("mpc.weight_accel", config_.weights.r_accel);
    get_parameter("mpc.weight_steer_rate", config_.weights.rd_steer);
    get_parameter("mpc.weight_accel_rate", config_.weights.rd_accel);

    get_parameter("mpc.max_steer_rad", config_.limits.max_steer_rad);
    get_parameter("mpc.max_steer_rate", config_.limits.max_steer_rate);
    get_parameter("mpc.min_accel", config_.limits.min_accel);
    get_parameter("mpc.max_accel", config_.limits.max_accel);

    get_parameter("steering.neutral", steering_calib_.neutral);
    get_parameter("steering.units_per_degree", steering_calib_.units_per_degree);
    get_parameter("steering.min_raw", steering_calib_.min_raw);
    get_parameter("steering.max_raw", steering_calib_.max_raw);
    get_parameter("safety.state_source_age_tolerance_sec", state_source_age_tolerance_sec_);
    get_parameter("path.reference_speed_default", reference_speed_default_);

    get_parameter("topics.vehicle_state", config_.topics.vehicle_state);
    get_parameter("topics.path", config_.topics.path);
    get_parameter("topics.stop", config_.topics.stop);
    get_parameter("topics.vehicle_command", config_.topics.vehicle_command);
    get_parameter("topics.predicted_path", config_.topics.predicted_path);
    get_parameter("topics.reference_path", config_.topics.reference_path);
}

void MpcControllerNode::SetupSubscribersAndPublishers() {
    state_sub_ = create_subscription<common_msgs::msg::HuatCarstate>(
        config_.topics.vehicle_state, 10,
        [this](const common_msgs::msg::HuatCarstate::ConstSharedPtr msg) { OnCarState(msg); });

    path_sub_ = create_subscription<common_msgs::msg::HuatPathLimits>(
        config_.topics.path, 10, [this](const common_msgs::msg::HuatPathLimits::ConstSharedPtr msg) { OnPath(msg); });

    const auto stop_qos = common_msgs::contract::makeQoS(common_msgs::contract::kQosStop);
    stop_sub_ = create_subscription<common_msgs::msg::HuatStop>(
        config_.topics.stop, stop_qos, [this](const common_msgs::msg::HuatStop::ConstSharedPtr msg) { OnStop(msg); });

    cmd_pub_ = create_publisher<common_msgs::msg::HuatVehicleCmd>(config_.topics.vehicle_command, 1);
    pred_path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.topics.predicted_path, 1);
    ref_path_pub_ = create_publisher<nav_msgs::msg::Path>(config_.topics.reference_path, 1);
}

void MpcControllerNode::OnCarState(const common_msgs::msg::HuatCarstate::ConstSharedPtr& msg) {
    // issue #12：来源年龄验证叠加在接收活性（last_state_time_）之外，防持续到达的延迟/旧戳状态刷新看门狗
    if (state_source_age_tolerance_sec_ >= 0.0) {
        const double source_age = (now() - msg->header.stamp).seconds();
        if (source_age > state_source_age_tolerance_sec_ || source_age < -state_source_age_tolerance_sec_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[MPC] Stale/invalid vehicle_state (source age %.3fs, tol %.3fs), ignored", source_age,
                                 state_source_age_tolerance_sec_);
            return;
        }
    }
    current_x_ = msg->car_state.x;
    current_y_ = msg->car_state.y;
    current_theta_ = msg->car_state.theta;
    current_speed_ = msg->v;
    last_state_time_ = now();
    has_state_ = true;
}

void MpcControllerNode::OnPath(const common_msgs::msg::HuatPathLimits::ConstSharedPtr& msg) {
    // 坐标系门禁（issue #3/#14）：未知/缺失 frame_id 不得静默当作 map，直接拒绝并进入降级（has_path_=false → 急停）
    const std::string& frame = msg->header.frame_id;
    if (!common_msgs::contract::isFrameSupported(frame)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[MPC] Unsupported path frame '%s' (expect 'map' or 'base_link'), path rejected!",
                             frame.c_str());
        has_path_ = false;
        return;
    }
    path_frame_ = frame;
    path_in_base_frame_ = (frame == common_msgs::contract::kFrameBaseLink);

    if (msg->path.size() < 2) {
        return;
    }

    std::vector<ReferencePoint> pts;
    pts.reserve(msg->path.size());
    const bool has_explicit_speeds = common_msgs::contract::targetSpeedsEffective(msg->target_speeds, msg->path.size());

    for (size_t i = 0; i < msg->path.size(); ++i) {
        ReferencePoint pt;
        pt.x = msg->path[i].x;
        pt.y = msg->path[i].y;
        // #14：速度载体唯一为 target_speeds；缺失时回退到配置的默认参考速度，绝不读 Point.z。
        const double target_speed = has_explicit_speeds ? msg->target_speeds[i] : 0.0;
        const auto speed =
            common_msgs::contract::selectReferenceSpeed(has_explicit_speeds, target_speed, reference_speed_default_);
        pt.speed = speed.speed;
        pt.speed_valid = speed.valid;

        // 计算航向角与曲率估计
        if (i + 1 < msg->path.size()) {
            double dx = msg->path[i + 1].x - pt.x;
            double dy = msg->path[i + 1].y - pt.y;
            pt.theta = std::atan2(dy, dx);
        } else if (!pts.empty()) {
            pt.theta = pts.back().theta;
        }

        if (i > 0 && i + 1 < msg->path.size()) {
            double dtheta = MpcModel::NormalizeAngle(pt.theta - pts.back().theta);
            double ds = std::hypot(msg->path[i].x - msg->path[i - 1].x, msg->path[i].y - msg->path[i - 1].y);
            pt.curvature = (ds > 1e-3) ? (dtheta / ds) : 0.0;
        } else {
            pt.curvature = 0.0;
        }

        pts.push_back(pt);
    }

    reference_path_ = std::move(pts);
    last_path_time_ = now();
    has_path_ = true;
}

void MpcControllerNode::OnStop(const common_msgs::msg::HuatStop::ConstSharedPtr& msg) {
    stop_requested_ = msg->stop;
}

void MpcControllerNode::ControlLoop() {
    rclcpp::Time current_time = now();

    // 1. 安全看门狗检查
    if (!has_state_ || !has_path_) {
        PublishEmergencyBrake();
        return;
    }

    if ((current_time - last_state_time_).seconds() > 0.5 || (current_time - last_path_time_).seconds() > 1.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[MPC] Timeout on state or path, braking!");
        PublishEmergencyBrake();
        return;
    }

    if (stop_requested_) {
        PublishEmergencyBrake();
        return;
    }

    // 2. 执行 MPC 优化计算：局部路径契约下车辆位姿变换到路径参考系（原点、航向 0），与 PP 同语义
    const double ref_x = path_in_base_frame_ ? 0.0 : current_x_;
    const double ref_y = path_in_base_frame_ ? 0.0 : current_y_;
    const double ref_theta = path_in_base_frame_ ? 0.0 : current_theta_;
    auto solution =
        mpc_model_.Step(ref_x, ref_y, ref_theta, current_speed_, prev_steer_rad_, prev_accel_mps2_, reference_path_);

    if (solution.success) {
        prev_steer_rad_ = solution.steering_rad;
        prev_accel_mps2_ = solution.accel_mps2;
        last_solve_time_ms_ = solution.solve_time_ms;

        PublishVehicleCommand(solution.steering_rad, solution.accel_mps2);
        PublishPredictedPath(solution.predicted_trajectory);
        PublishReferencePath(solution.reference_horizon);
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[MPC] Optimization did not converge, executing safety hold");
        PublishVehicleCommand(prev_steer_rad_ * 0.9, -1.0);
    }
}

void MpcControllerNode::PublishVehicleCommand(double steering_rad, double accel_mps2) {
    common_msgs::msg::HuatVehicleCmd cmd;
    cmd.head1 = 0xAA;
    cmd.head2 = 0x55;
    cmd.length = 10;
    cmd.gear_position = 1;
    cmd.working_mode = 1;
    cmd.racing_num = static_cast<uint8_t>(config_.system.racing_num);
    cmd.racing_status = 1;

    // 前轮转角映射统一走 SteeringCalibration（零位/比例/限幅均由 steering.* 参数配置）
    cmd.steering = static_cast<uint8_t>(steering_calib_.encodeRad(steering_rad));

    // 加速度映射为油门 (pedal_ratio) 与制动 (brake_force)
    if (accel_mps2 >= 0.0) {
        double throt_ratio = (accel_mps2 / config_.limits.max_accel) * 100.0;
        cmd.pedal_ratio = static_cast<uint8_t>(std::clamp(static_cast<int>(throt_ratio), 0, 100));
        cmd.brake_force = 0;
    } else {
        double brake_ratio = (-accel_mps2 / std::abs(config_.limits.min_accel)) * 100.0;
        cmd.pedal_ratio = 0;
        cmd.brake_force = static_cast<uint8_t>(std::clamp(static_cast<int>(brake_ratio), 0, 100));
    }

    cmd.checksum = static_cast<uint16_t>(cmd.steering + cmd.brake_force + cmd.pedal_ratio + cmd.gear_position +
                                         cmd.working_mode + cmd.racing_num + cmd.racing_status);
    cmd_pub_->publish(cmd);
}

void MpcControllerNode::PublishEmergencyBrake() {
    common_msgs::msg::HuatVehicleCmd cmd;
    cmd.head1 = 0xAA;
    cmd.head2 = 0x55;
    cmd.length = 10;
    cmd.gear_position = 1;
    cmd.working_mode = 1;
    cmd.racing_num = static_cast<uint8_t>(config_.system.racing_num);
    cmd.racing_status = 4;
    cmd.steering = static_cast<uint8_t>(steering_calib_.neutralRaw());
    cmd.pedal_ratio = 0;
    cmd.brake_force = 80;
    cmd.checksum = static_cast<uint16_t>(cmd.steering + cmd.brake_force + cmd.pedal_ratio + cmd.gear_position +
                                         cmd.working_mode + cmd.racing_num + cmd.racing_status);
    cmd_pub_->publish(cmd);
}

void MpcControllerNode::PublishPredictedPath(const std::vector<PredictedPoint>& trajectory) {
    nav_msgs::msg::Path path;
    path.header.frame_id = path_frame_;  // 预测轨迹与参考路径同系，RViz 展示与实际数值一致（issue #3）
    path.header.stamp = now();

    for (const auto& pt : trajectory) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.1;
        path.poses.push_back(pose);
    }

    pred_path_pub_->publish(path);
}

void MpcControllerNode::PublishReferencePath(const std::vector<ReferencePoint>& reference) {
    nav_msgs::msg::Path path;
    path.header.frame_id = path_frame_;
    path.header.stamp = now();

    for (const auto& pt : reference) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.05;
        path.poses.push_back(pose);
    }

    ref_path_pub_->publish(path);
}

}  // namespace mpc
