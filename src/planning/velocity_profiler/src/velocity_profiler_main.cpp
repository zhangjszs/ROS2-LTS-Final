#include <rclcpp/rclcpp.hpp>

#include "velocity_profiler/velocity_profiler_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<velocity_profiler::VelocityProfilerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
