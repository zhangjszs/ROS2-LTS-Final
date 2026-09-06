#include <rclcpp/rclcpp.hpp>

#include "straight_line_planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("straight_line_planner");

    StraightLinePlannerNode planner_node(node);
    planner_node.Run();

    return 0;
}
