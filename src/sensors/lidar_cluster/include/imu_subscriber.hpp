#pragma once

#include <common_msgs/msg/huat_ins_p2.hpp>

#include <Eigen/Dense>
#include <imu_data.hpp>
class ImuSubscriber {
   public:
    explicit ImuSubscriber(rclcpp::Node* node, std::string topic_name, size_t buff_size);
    explicit ImuSubscriber(rclcpp::Node& node, std::string topic_name, size_t buff_size)
        : ImuSubscriber(&node, std::move(topic_name), buff_size) {}
    explicit ImuSubscriber(const rclcpp::Node::SharedPtr& node, std::string topic_name, size_t buff_size)
        : ImuSubscriber(node.get(), std::move(topic_name), buff_size) {}
    ImuSubscriber() = default;
    bool buff_mutex_;

    void ParseData(std::deque<ImuData>& imu_data_buff);
    void SetInsInfo(common_msgs::msg::HuatInsP2::ConstSharedPtr imu_msg_ptr);
    bool SyncData(std::deque<ImuData>& UnsyncedData, ImuData& synced_data, double sync_time);
    bool readAndSync(std::deque<ImuData>& unsynced_imu_, ImuData& synced_data, double sync_time);

   private:
    rclcpp::Node* node_{nullptr};
    rclcpp::Subscription<common_msgs::msg::HuatInsP2>::SharedPtr subscriber_;
    std::deque<ImuData> imu_data_buff_;
    std::deque<ImuData> new_imu_data_;
    ImuData pre_imu_data;
    std::mutex mtx;
    void MsgCallback(const common_msgs::msg::HuatInsP2::ConstSharedPtr imu_msg_ptr);
    Eigen::Matrix3f GetOrientationMatrix();
};