#include <imu_subscriber.hpp>

#include <numbers>
ImuSubscriber::ImuSubscriber(rclcpp::Node* node, std::string topic_name, size_t buff_size) : node_(node) {
    subscriber_ = node_->create_subscription<common_msgs::msg::HuatInsP2>(
        topic_name, buff_size,
        [this](const common_msgs::msg::HuatInsP2::ConstSharedPtr msg) { MsgCallback(msg); });
}

void ImuSubscriber::MsgCallback(const common_msgs::msg::HuatInsP2::ConstSharedPtr imu_msg_ptr) {
    ImuData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.sec + imu_msg_ptr->header.stamp.nanosec * 1e-9;
    imu_data.linear_acceleration.x = imu_msg_ptr->vn;
    imu_data.linear_acceleration.y = imu_msg_ptr->ve;
    imu_data.linear_acceleration.z = imu_msg_ptr->vu;
    imu_data.angular_velocity.x = imu_msg_ptr->gyro_x;
    imu_data.angular_velocity.y = imu_msg_ptr->gyro_y;
    imu_data.angular_velocity.z = imu_msg_ptr->gyro_z;
    imu_data.rpy.heading = imu_msg_ptr->heading;
    imu_data.rpy.pitch = imu_msg_ptr->pitch;
    std::scoped_lock lock(mtx);
    new_imu_data_.push_back(imu_data);
}

void ImuSubscriber::ParseData(std::deque<ImuData> &imu_data_buff) {
    std::scoped_lock lock(mtx);
    if (!new_imu_data_.empty()) {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }
}

bool ImuSubscriber::SyncData(std::deque<ImuData> &UnsyncedData, ImuData &synced_data, double sync_time) {
    ImuData tmp_imu_data;
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }

        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    ImuData front_data = UnsyncedData.at(0);
    ImuData back_data = UnsyncedData.at(1);

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_acceleration.x =
        front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y =
        front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z =
        front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x =
        front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y =
        front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z =
        front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    synced_data.rpy.heading = front_data.rpy.heading * front_scale + back_data.rpy.heading * back_scale;
    synced_data.rpy.pitch = front_data.rpy.pitch * front_scale + back_data.rpy.pitch * back_scale;
    tmp_imu_data = synced_data;
    double dh = synced_data.rpy.heading - pre_imu_data.rpy.heading;
    // 归一化到 [-π, π] 防止跨越 ±π 边界时产生 ~360° 跳变
    constexpr double kPi = std::numbers::pi_v<double>;
    while (dh > kPi)
        dh -= 2.0 * kPi;
    while (dh < -kPi)
        dh += 2.0 * kPi;
    tmp_imu_data.rpy.heading = dh;
    tmp_imu_data.rpy.pitch = synced_data.rpy.pitch - pre_imu_data.rpy.pitch;
    pre_imu_data = synced_data;
    synced_data = tmp_imu_data;
    return true;
}
