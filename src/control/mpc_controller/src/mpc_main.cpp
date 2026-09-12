#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mpc_controller/mpc_controller_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mpc::MpcControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
