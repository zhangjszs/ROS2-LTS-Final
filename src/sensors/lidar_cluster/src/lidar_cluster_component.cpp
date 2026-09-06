#include <lidar_cluster.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <imu_subscriber.hpp>
#include <string>
#include <thread>

namespace lidar_cluster {
class LidarClusterComponent : public rclcpp::Node {
   public:
    LidarClusterComponent() : LidarClusterComponent(rclcpp::NodeOptions()) {}

    explicit LidarClusterComponent(const rclcpp::NodeOptions & options)
        : rclcpp::Node("lidar_cluster_node", options), running_(false) {
        lc_ = std::make_shared<LidarCluster>(shared_from_this());
        running_ = true;
        poll_thread_ = std::thread([this]() { algoPoll(); });
    }

    ~LidarClusterComponent() {
        running_ = false;
        if (lc_)
            lc_->RequestStop();
        if (poll_thread_.joinable())
            poll_thread_.join();
    }

   private:
    void algoPoll() {
        while (running_ && rclcpp::ok()) {
            lc_->WaitForPendingCloud();
            if (!running_ || !rclcpp::ok())
                break;
            lc_->RunAlgorithm();
        }
        running_ = false;
    }

    volatile bool running_;
    std::thread poll_thread_;
    std::shared_ptr<LidarCluster> lc_;
};

}  // namespace lidar_cluster

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_cluster::LidarClusterComponent)
