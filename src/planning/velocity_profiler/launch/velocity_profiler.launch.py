import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('velocity_profiler')
    default_params = os.path.join(pkg_dir, 'config', 'velocity_profiler_params.yaml')

    profiler_node = Node(
        package='velocity_profiler',
        executable='velocity_profiler_node',
        name='velocity_profiler_node',
        output='screen',
        parameters=[default_params]
    )

    return LaunchDescription([profiler_node])
