import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_pkg_dir = get_package_share_directory('vehicle_simulator')

    sim_skidpad_file = (
        os.path.join(sim_pkg_dir, 'launch', 'sim_skidpad.launch.py')
        if os.path.exists(os.path.join(sim_pkg_dir, 'launch', 'sim_skidpad.launch.py'))
        else os.path.join(sim_pkg_dir, 'sim_skidpad.launch.py')
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true' if os.environ.get('DISPLAY') else 'false',
        description='Whether to launch RViz2'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_skidpad_file),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()
    )

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
        use_rviz_arg,
        sim_launch,
        benchmark_node
    ])
