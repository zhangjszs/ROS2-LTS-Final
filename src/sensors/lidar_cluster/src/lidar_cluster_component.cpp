#include <lidar_cluster.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <imu_subscriber.hpp>
#include <stop_token>
#include <string>
#include <thread>

namespace lidar_cluster {
class LidarClusterComponent : public rclcpp::Node {
   public:
    LidarClusterComponent() : LidarClusterComponent(rclcpp::NodeOptions()) {}

    explicit LidarClusterComponent(const rclcpp::NodeOptions & options)
        : rclcpp::Node("lidar_cluster_node", options) {
        lc_ = std::make_shared<LidarCluster>(shared_from_this());
        poll_thread_ = std::jthread([this](std::stop_token st) { algoPoll(st); });
    }

    ~LidarClusterComponent() override {
        if (lc_)
            lc_->RequestStop();
        // C++20 std::jthread 析构时通过 RAII 自动触发 request_stop() 并自动 join()，杜绝未 join 导致的崩溃
    }

   private:
    void algoPoll(std::stop_token st) {
        while (!st.stop_requested() && rclcpp::ok()) {
            lc_->WaitForPendingCloud();
            if (st.stop_requested() || !rclcpp::ok())
                break;
            lc_->RunAlgorithm();
        }
    }

    std::jthread poll_thread_;
    std::shared_ptr<LidarCluster> lc_;
};

}  // namespace lidar_cluster

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_cluster::LidarClusterComponent)
