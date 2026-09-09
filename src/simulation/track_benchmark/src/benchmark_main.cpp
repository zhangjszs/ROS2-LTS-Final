#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "track_benchmark/benchmark_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<benchmark::BenchmarkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
