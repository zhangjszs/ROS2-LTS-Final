# /**
#  * \file nodelet_test.launch.py
#  * \brief Nodelet pipeline test
#  *
#  * Converts nodelet_test.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Note: ROS1 nodelets become ROS2 components. This launch file
#  * launches the equivalent component container and components.
#  *
#  * Usage:
#  *   ros2 launch huat_launch nodelet_test.launch.py
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

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # ROS2 component container (replaces nodelet manager)
    component_container = Node(
        package='rclcpp_components',
        executable='component_container',
        name='nodelet_manager',
        output='screen',
    )

    # Load SampleNodelet components into the container
    # In ROS2, components are loaded via the component_container CLI or service
    # Here we use ExecuteProcess to load them after the container starts
    load_node_1 = ExecuteProcess(
        cmd=[
            'ros2', 'component', 'load',
            '/nodelet_manager',
            'nodelet_test',
            'nodelet_test::SampleNodelet',
            'Node_1',
        ],
        output='screen',
    )

    load_node_2 = ExecuteProcess(
        cmd=[
            'ros2', 'component', 'load',
            '/nodelet_manager',
            'nodelet_test',
            'nodelet_test::SampleNodelet',
            'Node_2',
        ],
        output='screen',
    )

    load_node_3 = ExecuteProcess(
        cmd=[
            'ros2', 'component', 'load',
            '/nodelet_manager',
            'nodelet_test',
            'nodelet_test::SampleNodelet',
            'Node_3',
        ],
        output='screen',
    )

    return LaunchDescription([
        component_container,
        load_node_1,
        load_node_2,
        load_node_3,
    ])
