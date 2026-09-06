# /**
#  * \file sensor_stack.launch.py
#  * \brief Sensor stack: Velodyne -> lidar_cluster -> vehicle_state -> cone_fusion -> cone_dedup
#  *
#  * Converts sensor_stack.launch (ROS1) to ROS2 Python launch file.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
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
    standardized_ins_topic_arg = DeclareLaunchArgument(
        'standardized_ins_topic',
        default_value='/sensors/ins/raw',
        description='Standardized INS topic',
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
    lidar_ins_p2_topic_arg = DeclareLaunchArgument(
        'lidar_ins_p2_topic',
        default_value='/sensors/ins_p2',
        description='LiDAR INS P2 topic',
    )
    enable_lidar_debug_topics_arg = DeclareLaunchArgument(
        'enable_lidar_debug_topics',
        default_value='false',
        description='Enable LiDAR debug topics',
    )
    enable_velodyne_driver_arg = DeclareLaunchArgument(
        'enable_velodyne_driver',
        default_value='true',
        description='Enable Velodyne driver (disable for rosbag replay)',
    )
    lidar_road_type_arg = DeclareLaunchArgument(
        'lidar_road_type',
        default_value='3',
        description='LiDAR road type',
    )
    lidar_x_arg = DeclareLaunchArgument(
        'lidar_x',
        default_value='1.87',
        description='LiDAR extrinsic X',
    )
    lidar_y_arg = DeclareLaunchArgument(
        'lidar_y',
        default_value='0.0',
        description='LiDAR extrinsic Y',
    )
    lidar_z_arg = DeclareLaunchArgument(
        'lidar_z',
        default_value='0.135',
        description='LiDAR extrinsic Z',
    )
    lidar_yaw_arg = DeclareLaunchArgument(
        'lidar_yaw',
        default_value='0.0',
        description='LiDAR extrinsic yaw (rad)',
    )
    lidar_pitch_arg = DeclareLaunchArgument(
        'lidar_pitch',
        default_value='0.0',
        description='LiDAR extrinsic pitch (rad)',
    )
    lidar_roll_arg = DeclareLaunchArgument(
        'lidar_roll',
        default_value='0.0',
        description='LiDAR extrinsic roll (rad)',
    )

    # Static transform: base_link -> velodyne
    # Always publish, including during rosbag replay when Velodyne driver is off
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_velodyne',
        arguments=[
            LaunchConfiguration('lidar_x'),
            LaunchConfiguration('lidar_y'),
            LaunchConfiguration('lidar_z'),
            LaunchConfiguration('lidar_yaw'),
            LaunchConfiguration('lidar_pitch'),
            LaunchConfiguration('lidar_roll'),
            'base_link',
            'velodyne',
        ],
    )

    # Velodyne VLP-32C driver (disabled during rosbag replay)
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne_pointcloud'),
                'launch',
                'VLP-32C_points.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('enable_velodyne_driver')),
    )

    # INS relay: raw INS -> standardized namespace
    # ROS2 topic_tools provides relay functionality
    ins_relay_node = Node(
        package='topic_tools',
        executable='relay_node',
        name='ins_relay',
        arguments=[
            LaunchConfiguration('raw_ins_topic'),
            LaunchConfiguration('standardized_ins_topic'),
        ],
        remappings=[
            (LaunchConfiguration('raw_ins_topic'), LaunchConfiguration('raw_ins_topic')),
            (LaunchConfiguration('standardized_ins_topic'), LaunchConfiguration('standardized_ins_topic')),
        ],
    )

    # LiDAR cluster (component)
    lidar_cluster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lidar_cluster'),
                'launch',
                'lidar_cluster.launch.py',
            )
        ),
        launch_arguments={
            'input_topic': LaunchConfiguration('pointcloud_topic'),
            'output_cones_topic': LaunchConfiguration('raw_cones_topic'),
            'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            'ins_p2_topic': LaunchConfiguration('lidar_ins_p2_topic'),
            'ins_asensing_topic': LaunchConfiguration('raw_ins_topic'),
            'enable_debug_topics': LaunchConfiguration('enable_lidar_debug_topics'),
            'road_type': LaunchConfiguration('lidar_road_type'),
        }.items(),
    )

    # Vehicle state estimation
    vehicle_state_config = os.path.join(
        get_package_share_directory('vehicle_state'),
        'config',
        'vehicle_state.yaml',
    )
    vehicle_state_node = Node(
        package='vehicle_state',
        executable='vehicle_state',
        name='vehicle_state',
        output='screen',
        parameters=[
            vehicle_state_config,
            {
                'ins_topic': LaunchConfiguration('raw_ins_topic'),
                'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            },
        ],
    )

    # Cone fusion: lidar frame -> global frame
    cone_fusion_config = os.path.join(
        get_package_share_directory('cone_fusion'),
        'config',
        'cone_fusion.yaml',
    )
    cone_fusion_node = Node(
        package='cone_fusion',
        executable='cone_fusion_node',
        name='cone_fusion',
        output='screen',
        parameters=[
            cone_fusion_config,
            {
                'input_cones_topic': LaunchConfiguration('raw_cones_topic'),
                'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
                'transformed_cones_topic': LaunchConfiguration('transformed_cones_topic'),
                'global_map_topic': LaunchConfiguration('global_map_topic'),
            },
        ],
    )

    # Cone tracker: KD-tree matching + EMA + Kalman filter + sliding window
    cone_dedup_config = os.path.join(
        get_package_share_directory('cone_tracker'),
        'config',
        'cone_dedup.yaml',
    )
    cone_tracker_node = Node(
        package='cone_tracker',
        executable='cone_tracker_node',
        name='cone_tracker',
        output='screen',
        parameters=[
            cone_dedup_config,
            {
                'transformed_cones_topic': LaunchConfiguration('transformed_cones_topic'),
                'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
                'fused_cones_topic': LaunchConfiguration('fused_cones_topic'),
            },
        ],
    )

    return LaunchDescription([
        # Arguments
        pointcloud_topic_arg,
        raw_ins_topic_arg,
        standardized_ins_topic_arg,
        vehicle_state_topic_arg,
        raw_cones_topic_arg,
        transformed_cones_topic_arg,
        fused_cones_topic_arg,
        global_map_topic_arg,
        lidar_ins_p2_topic_arg,
        enable_lidar_debug_topics_arg,
        enable_velodyne_driver_arg,
        lidar_road_type_arg,
        lidar_x_arg,
        lidar_y_arg,
        lidar_z_arg,
        lidar_yaw_arg,
        lidar_pitch_arg,
        lidar_roll_arg,

        # Static TF
        static_tf_node,

        # Velodyne driver (conditional)
        velodyne_launch,

        # INS relay
        ins_relay_node,

        # LiDAR cluster
        lidar_cluster_launch,

        # Vehicle state
        vehicle_state_node,

        # Cone fusion
        cone_fusion_node,

        # Cone tracker
        cone_tracker_node,
    ])
