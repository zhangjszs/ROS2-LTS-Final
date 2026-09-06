#include <rclcpp/rclcpp.hpp>

#include "skidpad_planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("skidpad_planner");

    skidpad::SkidpadPlannerNode planner_node(node);
    planner_node.Run();

    return 0;
}
