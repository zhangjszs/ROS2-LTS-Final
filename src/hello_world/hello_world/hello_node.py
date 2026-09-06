#!/usr/bin/env python3
"""最简 ROS 2 Hello World 节点 - 定时打印日志。"""
import rclpy
from rclpy.node import Node


class HelloWorldNode(Node):
    def __init__(self) -> None:
        super().__init__('hello_world_node')
        self.counter = 0
        # 每 0.5 秒触发一次定时器
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Hello World 节点已启动！')

    def timer_callback(self) -> None:
        self.counter += 1
        self.get_logger().info(f'Hello World [{self.counter}]')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HelloWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001 - 优雅退出，避免 timeout SIGTERM 时打印堆栈
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
