#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vehicle_simulator/simulator_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simulation::SimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
