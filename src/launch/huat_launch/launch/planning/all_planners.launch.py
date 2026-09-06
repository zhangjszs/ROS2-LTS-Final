# /**
#  * \file all_planners.launch.py
#  * \brief All planners with topic multiplexer for runtime hot-switching
#  *
#  * Converts all_planners.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Mux output: /planning/pathlimits
#  * Switch: ros2 service call /planner_mux/select topic_tools/srv/MuxSelect "{topic: '/planning/track/pathlimits'}"
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
    fused_cones_topic_arg = DeclareLaunchArgument(
        'fused_cones_topic',
        default_value='/sensors/cones/fused',
        description='Fused cone map topic',
    )
    vehicle_state_topic_arg = DeclareLaunchArgument(
        'vehicle_state_topic',
        default_value='/localization/vehicle_state',
        description='Vehicle state topic',
    )
    mux_output_topic_arg = DeclareLaunchArgument(
        'mux_output_topic',
        default_value='/planning/pathlimits',
        description='Mux output path topic',
    )
    skidpad_pathlimits_topic_arg = DeclareLaunchArgument(
        'skidpad_pathlimits_topic',
        default_value='/planning/skidpad/pathlimits',
        description='Skidpad path topic',
    )
    straight_line_pathlimits_topic_arg = DeclareLaunchArgument(
        'straight_line_pathlimits_topic',
        default_value='/planning/straight_line/pathlimits',
        description='Straight line path topic',
    )
    track_pathlimits_topic_arg = DeclareLaunchArgument(
        'track_pathlimits_topic',
        default_value='/planning/track/pathlimits',
        description='Track path topic',
    )
    track_stop_request_topic_arg = DeclareLaunchArgument(
        'track_stop_request_topic',
        default_value='/planning/track/stop_request',
        description='Track stop request topic',
    )
    mux_initial_topic_arg = DeclareLaunchArgument(
        'mux_initial_topic',
        default_value=LaunchConfiguration('track_pathlimits_topic'),
        description='Initial mux topic',
    )

    # Skidpad planner
    skidpad_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'planning',
                'skidpad.launch.py',
            )
        ),
        launch_arguments={
            'input_cone_map_topic': LaunchConfiguration('fused_cones_topic'),
            'output_pathlimits_topic': LaunchConfiguration('skidpad_pathlimits_topic'),
        }.items(),
    )

    # Straight line planner
    straight_line_launch = IncludeLaunchDescription(
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
            'output_pathlimits_topic': LaunchConfiguration('straight_line_pathlimits_topic'),
            'vehicle_state_topic': LaunchConfiguration('vehicle_state_topic'),
        }.items(),
    )

    # Track planner
    track_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'planning',
                'track.launch.py',
            )
        ),
        launch_arguments={
            'input_cones_topic': LaunchConfiguration('fused_cones_topic'),
            'input_pose_topic': LaunchConfiguration('vehicle_state_topic'),
            'output_topic': LaunchConfiguration('track_pathlimits_topic'),
            'stop_topic': LaunchConfiguration('track_stop_request_topic'),
        }.items(),
    )

    # Topic multiplexer: selects which planner output goes to controller
    # ROS2 topic_tools provides a mux node
    planner_mux_node = Node(
        package='topic_tools',
        executable='mux_node',
        name='planner_mux',
        output='screen',
        arguments=[
            LaunchConfiguration('mux_output_topic'),
            LaunchConfiguration('skidpad_pathlimits_topic'),
            LaunchConfiguration('straight_line_pathlimits_topic'),
            LaunchConfiguration('track_pathlimits_topic'),
        ],
        parameters=[{
            'initial_topic': LaunchConfiguration('mux_initial_topic'),
        }],
        remappings=[
            # topic_tools mux expects input topics as remappings
            (LaunchConfiguration('mux_output_topic'), LaunchConfiguration('mux_output_topic')),
        ],
    )

    return LaunchDescription([
        # Arguments
        fused_cones_topic_arg,
        vehicle_state_topic_arg,
        mux_output_topic_arg,
        skidpad_pathlimits_topic_arg,
        straight_line_pathlimits_topic_arg,
        track_pathlimits_topic_arg,
        track_stop_request_topic_arg,
        mux_initial_topic_arg,

        # Planners
        skidpad_launch,
        straight_line_launch,
        track_launch,

        # Mux
        planner_mux_node,
    ])
