/**
 * 订阅/INS/ASENSING_INS消息;
 * 处理消息得到以车身起始状态为坐标轴的车辆位置和航线角等信息;
 * 车辆可视化；
 * 发布话题/Carstate传递给cone_position处理锥筒坐标
 */
#include "vehicle_state_estimator.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("vehicle_state");
    RCLCPP_INFO(node->get_logger(), "[vehicle_state] Node started");
    VehicleStateEstimator estimator(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
        executor.spin_some();
        estimator.UpdateDiagnostics();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
