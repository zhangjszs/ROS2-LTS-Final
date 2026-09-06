# /**
#  * \file vision.launch.py
#  * \brief FSD vision subsystem: image cone detection and color classification
#  *
#  * Converts vision.launch (ROS1) to ROS2 Python launch file.
#  *
#  * Args:
#  *   mission: track | skidpad | accel (select mission overlay config)
#  *   mode: standard | lightweight | fallback_only
#  *   debug: true/false (enable debug image publishing)
#  *   rviz: true/false (launch RViz visualization)
#  *   visualize: true/false (launch cone_visualizer OpenCV window)
#  *   image_topic: image topic (shared by vision_node and visualizer)
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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='track',
        description='Mission type (track|skidpad|accel)',
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='standard',
        description='Mode (standard|lightweight|fallback_only)',
    )
    impl_arg = DeclareLaunchArgument(
        'impl',
        default_value='py',
        description='Vision implementation (cpp|py)',
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug image publishing',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization',
    )
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='false',
        description='Launch cone_visualizer OpenCV window',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Image topic',
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(
            get_package_share_directory('vision_ros'),
            'models',
            'exp26l_gpu',
            'best.onnx',
        ),
        description='ONNX model path',
    )
    require_model_arg = DeclareLaunchArgument(
        'require_model',
        default_value='true',
        description='Require model file',
    )
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='perception/vision',
        description='Namespace for vision nodes',
    )
    extra_config_arg = DeclareLaunchArgument(
        'extra_config',
        default_value='',
        description='Vehicle overlay config',
    )
    extra_local_config_arg = DeclareLaunchArgument(
        'extra_local_config',
        default_value='',
        description='Local override config',
    )
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Use sim time for rosbag playback',
    )
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value='',
        description='Rosbag file path for playback',
    )

    # Config file paths
    vision_base_config = os.path.join(
        get_package_share_directory('vision_ros'),
        'config',
        'vision_base.yaml',
    )
    vision_mission_config = os.path.join(
        get_package_share_directory('vision_ros'),
        'config',
        PythonExpression(["'vision_'", LaunchConfiguration("mission"), "'.yaml'"]),
    )

    # Vision node (Python implementation)
    vision_node = Node(
        package='vision_ros',
        executable='vision_node_py.py',
        name='vision_node',
        output='screen',
        namespace=LaunchConfiguration('ns'),
        parameters=[
            vision_base_config,
            vision_mission_config,
            # Extra config (conditional)
            LaunchConfiguration('extra_config'),
            # Extra local config (conditional)
            LaunchConfiguration('extra_local_config'),
            {
                # Mode overrides
                'inference/backend_type': 'fallback_only',
                'node/require_model': False,
                # Lightweight mode
                'inference/input_width': 320,
                'inference/input_height': 320,
                # Model path
                'inference/model_path': LaunchConfiguration('model_path'),
                'node/require_model': LaunchConfiguration('require_model'),
                # Debug
                'output/publish_debug_image': True,
                # Image topic
                'node/image_topic': LaunchConfiguration('image_topic'),
            },
        ],
        # Only use conditional parameters when conditions are met
        # In ROS2, we use PythonExpression for conditional parameter values
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('impl'), "' == 'py'"])
        ),
    )

    # Cone visualizer (OpenCV window)
    cone_visualizer_node = Node(
        package='vision_ros',
        executable='cone_visualizer.py',
        name='cone_visualizer',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'detection_topic': ['/', LaunchConfiguration('ns'), '/detections'],
        }],
        condition=IfCondition(LaunchConfiguration('visualize')),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('vision_ros'),
            'config',
            'vision_debug.rviz',
        )],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Rosbag playback
    rosbag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '--clock',
            LaunchConfiguration('bag'),
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('bag'), "' != ''"])
        ),
    )

    return LaunchDescription([
        # Arguments
        mission_arg,
        mode_arg,
        impl_arg,
        debug_arg,
        rviz_arg,
        visualize_arg,
        image_topic_arg,
        model_path_arg,
        require_model_arg,
        ns_arg,
        extra_config_arg,
        extra_local_config_arg,
        simulation_arg,
        bag_arg,

        # Nodes
        vision_node,
        cone_visualizer_node,
        rviz_node,
        rosbag_play_cmd,
    ])
