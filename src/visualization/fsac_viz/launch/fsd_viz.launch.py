# /**
#  * \file fsd_viz.launch.py
#  * \brief Unified FSD visualization node
#  *
#  * Converts fsd_viz.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Replaces separate cone_visualizer + vehicle_visualizer with fsd_viz_node.
#  * Publishes MarkerArray topics:
#  *   /fsd/viz/cones      - Cone detections with type-based coloring and confidence alpha
#  *   /fsd/viz/vehicle    - Vehicle body, trail, velocity arrow, status text
#  *   /fsd/viz/path       - Center path, track boundaries
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz',
    )
    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=os.path.join(
            get_package_share_directory('fsac_viz'),
            'config',
            'fsac_viz.rviz',
        ),
        description='RViz config file',
    )
    fixed_frame_arg = DeclareLaunchArgument(
        'fixed_frame',
        default_value='map',
        description='Fixed frame for visualization',
    )
    cone_map_topic_arg = DeclareLaunchArgument(
        'cone_map_topic',
        default_value='/sensors/cones/fused',
        description='Cone map topic',
    )
    cone_cluster_topic_arg = DeclareLaunchArgument(
        'cone_cluster_topic',
        default_value='/sensors/cones/raw',
        description='Cone cluster topic',
    )
    vehicle_state_topic_arg = DeclareLaunchArgument(
        'vehicle_state_topic',
        default_value='/localization/vehicle_state',
        description='Vehicle state topic',
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='/planning/pathlimits',
        description='Path topic',
    )

    # Unified visualization node
    fsd_viz_node = Node(
        package='fsac_viz',
        executable='fsd_viz_node',
        name='fsd_viz_node',
        output='screen',
        parameters=[{
            'fixed_frame': LaunchConfiguration('fixed_frame'),
            'cone_map_topic': LaunchConfiguration('cone_map_topic'),
            'cone_cluster_topic': LaunchConfiguration('cone_cluster_topic'),
            'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            'path_topic': LaunchConfiguration('path_topic'),
            'use_mesh': False,
            'cone_scale': 0.3,
            'min_alpha': 0.3,
            'show_confidence_alpha': True,
            'show_distance_label': True,
            'show_cluster_bbox': False,
            'marker_lifetime': 0.5,
            # Cone colors
            'color_blue': [0.0, 0.0, 1.0, 1.0],
            'color_yellow': [1.0, 0.85, 0.0, 1.0],
            'color_orange': [1.0, 0.5, 0.0, 1.0],
            'color_unknown': [0.8, 0.8, 0.8, 1.0],
            # Vehicle
            'car_scale': 0.001,
            'show_trail': True,
            'trail_max_points': 200,
            'trail_min_dist': 0.1,
            'show_velocity_arrow': True,
            'show_status_text': True,
            'show_heading_arrow': True,
            # Path
            'show_center_path': True,
            'show_boundaries': True,
            'show_path_points': True,
        }],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='fsac_viz_rviz',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        # Arguments
        rviz_arg,
        rviz_cfg_arg,
        fixed_frame_arg,
        cone_map_topic_arg,
        cone_cluster_topic_arg,
        vehicle_state_topic_arg,
        path_topic_arg,

        # Nodes
        fsd_viz_node,
        rviz_node,
    ])
