from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """同时启动 talker 和 listener，用于演示话题通信。"""
    return LaunchDescription([
        Node(
            package='hello_world',
            executable='talker',
            name='talker',
            output='screen',
        ),
        Node(
            package='hello_world',
            executable='listener',
            name='listener',
            output='screen',
        ),
        # 单纯的 Hello World 节点（可选，不加也行，这里注释掉避免刷屏）
        # Node(
        #     package='hello_world',
        #     executable='hello_node',
        #     name='hello_world_node',
        #     output='screen',
        # ),
    ])
