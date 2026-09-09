import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_pkg_dir = get_package_share_directory('vehicle_simulator')

    # 包含基础八字闭环仿真
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_dir, 'sim_skidpad.launch.py')
        )
    )

    # 启动 KPI 性能基准评测看板节点
    benchmark_node = Node(
        package='track_benchmark',
        executable='track_benchmark_node',
        name='track_benchmark_node',
        output='screen',
        parameters=[{
            'track_type': 'skidpad',
            'controller_name': 'PurePursuit',
            'report_file': 'benchmark_skidpad_pure_pursuit.md'
        }]
    )

    return LaunchDescription([
        sim_launch,
        benchmark_node
    ])
