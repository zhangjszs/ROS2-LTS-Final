# /**
#  * \file straight_line.launch.py
#  * \brief Straight line planner (acceleration, Hough+RANSAC)
#  *
#  * Converts straight_line.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Output: /planning/straight_line/pathlimits
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    input_cone_map_topic_arg = DeclareLaunchArgument(
        'input_cone_map_topic',
        default_value='/sensors/cones/fused',
        description='Input cone map topic',
    )
    output_pathlimits_topic_arg = DeclareLaunchArgument(
        'output_pathlimits_topic',
        default_value='/planning/straight_line/pathlimits',
        description='Output path limits topic',
    )
    vehicle_state_topic_arg = DeclareLaunchArgument(
        'vehicle_state_topic',
        default_value='/localization/vehicle_state',
        description='Vehicle state topic',
    )

    # Straight line planner config
    straight_line_config = os.path.join(
        get_package_share_directory('straight_line_planner'),
        'config',
        'straight_line_planner.yaml',
    )

    # Straight line planner node
    straight_line_planner_node = Node(
        package='straight_line_planner',
        executable='straight_line_planner',
        name='straight_line_planner',
        output='screen',
        parameters=[
            straight_line_config,
            {
                'road_type': 2,
                'input_cone_map_topic': LaunchConfiguration('input_cone_map_topic'),
                'output_pathlimits_topic': LaunchConfiguration('output_pathlimits_topic'),
                'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
            },
        ],
    )

    return LaunchDescription([
        input_cone_map_topic_arg,
        output_pathlimits_topic_arg,
        vehicle_state_topic_arg,
        straight_line_planner_node,
    ])
