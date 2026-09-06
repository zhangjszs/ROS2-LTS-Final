# /**
#  * \file straight_line_run.launch.py
#  * \brief Straight line run: sensor stack + straight line planner + control + safety + visualization
#  *
#  * Converts straight_line_run.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Usage:
#  *   ros2 launch huat_launch straight_line_run.launch.py
#  */
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/velodyne_points',
        description='Point cloud input topic',
    )
    raw_ins_topic_arg = DeclareLaunchArgument(
        'raw_ins_topic',
        default_value='/INS/ASENSING_INS',
        description='Raw INS topic',
    )
    vehicle_state_topic_arg = DeclareLaunchArgument(
        'vehicle_state_topic',
        default_value='/localization/vehicle_state',
        description='Vehicle state topic',
    )
    raw_cones_topic_arg = DeclareLaunchArgument(
        'raw_cones_topic',
        default_value='/sensors/cones/raw',
        description='Raw cone detections topic',
    )
    transformed_cones_topic_arg = DeclareLaunchArgument(
        'transformed_cones_topic',
        default_value='/sensors/cones/transformed',
        description='Transformed cones topic',
    )
    fused_cones_topic_arg = DeclareLaunchArgument(
        'fused_cones_topic',
        default_value='/sensors/cones/fused',
        description='Fused cone map topic',
    )
    global_map_topic_arg = DeclareLaunchArgument(
        'global_map_topic',
        default_value='/sensors/map/global',
        description='Global map topic',
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='/planning/straight_line/pathlimits',
        description='Straight line path topic',
    )
    stop_topic_arg = DeclareLaunchArgument(
        'stop_topic',
        default_value='/system/stop',
        description='Stop signal topic',
    )
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/control/vehicle_command',
        description='Vehicle command topic',
    )
    enable_velodyne_driver_arg = DeclareLaunchArgument(
        'enable_velodyne_driver',
        default_value='true',
        description='Enable Velodyne driver (disable for rosbag replay)',
    )

    # Sensor stack
    sensor_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'sensors',
                'sensor_stack.launch.py',
            )
        ),
        launch_arguments={
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'raw_ins_topic': LaunchConfiguration('raw_ins_topic'),
            'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            'raw_cones_topic': LaunchConfiguration('raw_cones_topic'),
            'transformed_cones_topic': LaunchConfiguration('transformed_cones_topic'),
            'fused_cones_topic': LaunchConfiguration('fused_cones_topic'),
            'global_map_topic': LaunchConfiguration('global_map_topic'),
            'enable_velodyne_driver': LaunchConfiguration('enable_velodyne_driver'),
            'lidar_road_type': '2',
        }.items(),
    )

    # Straight line planner
    straight_line_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'planning',
                'straight_line.launch.py',
            )
        ),
        launch_arguments={
            'input_cone_map_topic': LaunchConfiguration('fused_cones_topic'),
            'output_pathlimits_topic': LaunchConfiguration('path_topic'),
        }.items(),
    )

    # Pure pursuit controller
    pure_pursuit_config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'pure_pursuit.yaml',
    )
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            pure_pursuit_config,
            {
                'road_type': 2,
                'topics/path': LaunchConfiguration('path_topic'),
                'topics/vehicle_state': LaunchConfiguration('vehicle_state_topic'),
                'topics/stop': LaunchConfiguration('stop_topic'),
                'topics/vehicle_command': LaunchConfiguration('vehicle_command_topic'),
            },
        ],
    )

    # Safety monitor
    safety_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'safety',
                'safety_monitor.launch.py',
            )
        ),
        launch_arguments={
            'pathlimits_topic': LaunchConfiguration('path_topic'),
            'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            'stop_topic': LaunchConfiguration('stop_topic'),
            'stop_request_topic': '',
        }.items(),
    )

    # Unified visualization
    fsd_viz_node = Node(
        package='fsac_viz',
        executable='fsd_viz_node',
        name='fsd_viz_node',
        output='screen',
        parameters=[{
            'cone_map_topic': LaunchConfiguration('fused_cones_topic'),
            'cone_cluster_topic': LaunchConfiguration('raw_cones_topic'),
            'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            'path_topic': LaunchConfiguration('path_topic'),
            'fixed_frame': 'map',
            'ins_topic': LaunchConfiguration('raw_ins_topic'),
        }],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory('huat_launch'),
            'config',
            'tuxiang.rviz',
        )],
    )

    return LaunchDescription([
        # Arguments
        pointcloud_topic_arg,
        raw_ins_topic_arg,
        vehicle_state_topic_arg,
        raw_cones_topic_arg,
        transformed_cones_topic_arg,
        fused_cones_topic_arg,
        global_map_topic_arg,
        path_topic_arg,
        stop_topic_arg,
        vehicle_command_topic_arg,
        enable_velodyne_driver_arg,

        # Launches
        sensor_stack_launch,
        straight_line_planner_launch,

        # Nodes
        pure_pursuit_node,
        safety_monitor_launch,
        fsd_viz_node,
        rviz_node,
    ])
