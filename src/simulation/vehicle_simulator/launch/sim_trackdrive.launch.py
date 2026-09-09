import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_pkg_dir = get_package_share_directory('vehicle_simulator')
    urinay_pkg_dir = get_package_share_directory('urinay')
    pp_pkg_dir = get_package_share_directory('pure_pursuit')
    viz_pkg_dir = get_package_share_directory('fsac_viz')
    huat_launch_dir = get_package_share_directory('huat_launch')

    default_track = os.path.join(sim_pkg_dir, 'tracks', 'trackdrive_loop.csv')
    default_sim_params = os.path.join(sim_pkg_dir, 'config', 'simulator_params.yaml')
    default_urinay_params = os.path.join(urinay_pkg_dir, 'config', 'urinay.yml')
    default_pp_params = os.path.join(pp_pkg_dir, 'config', 'pure_pursuit.yaml')
    default_viz_params = os.path.join(viz_pkg_dir, 'config', 'fsd_viz.yaml')
    default_rviz_config = os.path.join(huat_launch_dir, 'config', 'tuxiang.rviz')

    # 1. 车辆与传感器运动学闭环仿真器
    sim_node = Node(
        package='vehicle_simulator',
        executable='vehicle_simulator_node',
        name='vehicle_simulator',
        output='screen',
        parameters=[
            default_sim_params,
            {
                'track_file': default_track,
                'init_x': 0.0,
                'init_y': 20.0,
                'init_theta': 3.14159265,
                'init_v': 0.0,
            }
        ]
    )

    # 2. 操控赛 Delaunay 规划器 (Urinay)
    urinay_node = Node(
        package='urinay',
        executable='urinay_exec',
        name='urinay',
        output='screen',
        parameters=[
            default_urinay_params,
            {
                'road_type': 3,
                'input_cones_topic': '/sensors/cones/fused',
                'input_pose_topic': '/localization/vehicle_state',
                'output_topic': '/planning/pathlimits',
                'stop_topic': '/system/stop',
            }
        ]
    )

    # 3. 纯跟踪控制器 (Pure Pursuit)
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            default_pp_params,
            {
                'topics.vehicle_state': '/localization/vehicle_state',
                'topics.path': '/planning/pathlimits',
                'topics.vehicle_command': '/vehicle_command',
                'topics.stop': '/system/stop',
            }
        ]
    )

    # 4. 统一 RViz 赛车与赛道可视化器
    viz_node = Node(
        package='fsac_viz',
        executable='fsd_viz_node',
        name='fsd_viz_node',
        output='screen',
        parameters=[
            default_viz_params,
            {
                'vehicle_state_topic': '/localization/vehicle_state',
                'cone_map_topic': '/sensors/cones/fused',
                'path_topic': '/planning/pathlimits',
                'show_trail': True,
                'fixed_frame': 'map',
            }
        ]
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config] if os.path.exists(default_rviz_config) else []
    )

    return LaunchDescription([
        sim_node,
        urinay_node,
        pure_pursuit_node,
        viz_node,
        rviz_node
    ])
