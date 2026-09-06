# /**
#  * \file lidar_cluster.launch.py
#  * \brief LiDAR cluster: point cloud -> cone detection
#  *
#  * Converts lidar_cluster.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Note: ROS1 nodelet becomes ROS2 component. The nodelet manager is
#  * replaced by a component container, and the nodelet is loaded as
#  * a component into that container.
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/velodyne_points',
        description='Input point cloud topic',
    )
    output_cones_topic_arg = DeclareLaunchArgument(
        'output_cones_topic',
        default_value='/sensors/cones/raw',
        description='Output cones topic',
    )
    vehicle_state_topic_arg = DeclareLaunchArgument(
        'vehicle_state_topic',
        default_value='/localization/vehicle_state',
        description='Vehicle state topic',
    )
    ins_p2_topic_arg = DeclareLaunchArgument(
        'ins_p2_topic',
        default_value='/sensors/ins_p2',
        description='INS P2 topic',
    )
    ins_asensing_topic_arg = DeclareLaunchArgument(
        'ins_asensing_topic',
        default_value='/INS/ASENSING_INS',
        description='ASENSING INS topic',
    )
    enable_debug_topics_arg = DeclareLaunchArgument(
        'enable_debug_topics',
        default_value='false',
        description='Enable debug topics',
    )
    road_type_arg = DeclareLaunchArgument(
        'road_type',
        default_value='3',
        description='Road type selector',
    )
    cluster_config_arg = DeclareLaunchArgument(
        'cluster_config',
        default_value=os.path.join(
            get_package_share_directory('lidar_cluster'),
            'config',
            'lidar_cluster.yaml',
        ),
        description='LiDAR cluster config file',
    )

    # Component container (replaces nodelet manager)
    component_container = Node(
        package='rclcpp_components',
        executable='component_container',
        name='lidar_cluster_nodelet_manager',
        output='screen',
    )

    # Load LidarClusterComponent as component
    # In ROS2, components are loaded via CLI after container starts
    load_lidar_cluster_component = ExecuteProcess(
        cmd=[
            'ros2', 'component', 'load',
            '/lidar_cluster_nodelet_manager',
            'lidar_cluster',
            'lidar_cluster::LidarClusterComponent',
            'lidar_cluster_node',
        ],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        input_topic_arg,
        output_cones_topic_arg,
        vehicle_state_topic_arg,
        ins_p2_topic_arg,
        ins_asensing_topic_arg,
        enable_debug_topics_arg,
        road_type_arg,
        cluster_config_arg,

        # Container and component
        component_container,
        load_lidar_cluster_component,
    ])
