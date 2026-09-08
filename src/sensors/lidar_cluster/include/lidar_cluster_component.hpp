#pragma once

#include <lidar_cluster.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stop_token>
#include <string>
#include <thread>

namespace lidar_cluster {

/**
 * @brief ROS 2 现代化组件节点：管理 LidarCluster 算法主循环生命周期与后台线程
 */
class LidarClusterComponent : public rclcpp::Node {
   public:
    LidarClusterComponent() : LidarClusterComponent(rclcpp::NodeOptions()) {}

    explicit LidarClusterComponent(const rclcpp::NodeOptions & options)
        : rclcpp::Node("lidar_cluster_node", options) {
        // C++20 现代架构改进：直接传递当前 Node 指针 (this) 传递非拥有观测权，彻底杜绝构造期调用 shared_from_this() 导致的 std::bad_weak_ptr 崩溃
        lc_ = std::make_unique<LidarCluster>(this);
        poll_thread_ = std::jthread([this](std::stop_token st) { algoPoll(st); });
    }

    ~LidarClusterComponent() override {
        if (lc_) {
            lc_->RequestStop();
        }
        if (poll_thread_.joinable()) {
            poll_thread_.request_stop();
            poll_thread_.join();
        }
    }

    [[nodiscard]] bool isRunning() const noexcept {
        return lc_ != nullptr;
    }

   private:
    void algoPoll(std::stop_token st) {
        while (!st.stop_requested() && rclcpp::ok()) {
            lc_->WaitForPendingCloud();
            if (st.stop_requested() || !rclcpp::ok()) {
                break;
            }
            lc_->RunAlgorithm();
        }
    }

    // 严谨架构声明顺序：lc_ 必须声明在 poll_thread_ 之前，确保析构时后台线程先 join 完成，再销毁 lc_ 资源
    std::unique_ptr<LidarCluster> lc_;
    std::jthread poll_thread_;
};

}  // namespace lidar_cluster
