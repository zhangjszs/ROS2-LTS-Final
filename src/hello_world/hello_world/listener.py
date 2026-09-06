#!/usr/bin/env python3
"""订阅者示例 - 订阅 /chatter 并打印收到的消息。"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self) -> None:
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # 避免未使用变量警告
        self.get_logger().info('Listener 已启动，正在监听 /chatter ...')

    def listener_callback(self, msg: String) -> None:
        self.get_logger().info(f'收到: "{msg.data}"')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
