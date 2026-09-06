#include "cone_dedup.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cone_dedup");
    ConeDedup dedup(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
