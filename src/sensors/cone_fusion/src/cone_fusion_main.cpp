#include "cone_fusion.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cone_fusion");
    ConeFusion fusion(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
