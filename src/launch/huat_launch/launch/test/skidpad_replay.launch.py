# /**
#  * \file skidpad_replay.launch.py
#  * \brief Skidpad replay: skidpad_run.launch + rosbag playback
#  *
#  * Converts skidpad_replay.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Vehicle commands are remapped to debug topic to prevent accidental vehicle movement.
#  *
#  * Usage:
#  *   ros2 launch huat_launch skidpad_replay.launch.py
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value=os.path.join(os.environ.get('HOME', ''), 'rosbag', 'skidpad.bag'),
        description='Path to rosbag for skidpad replay',
    )
    vehicle_command_topic_arg = DeclareLaunchArgument(
        'vehicle_command_topic',
        default_value='/debug/replay/vehicle_command',
        description='Vehicle command topic (remapped to debug)',
    )

    # Skidpad run launch (with velodyne driver disabled)
    skidpad_run_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('huat_launch'),
                'launch',
                'main',
                'skidpad_run.launch.py',
            )
        ),
        launch_arguments={
            'enable_velodyne_driver': 'false',
            'vehicle_command_topic': LaunchConfiguration('vehicle_command_topic'),
        }.items(),
    )

    # Rosbag playback with clock
    rosbag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '--clock',
            LaunchConfiguration('bag'),
        ],
        output='screen',
    )

    return LaunchDescription([
        bag_arg,
        vehicle_command_topic_arg,
        skidpad_run_launch,
        rosbag_play_cmd,
    ])
