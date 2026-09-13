import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_pkg_dir = get_package_share_directory('vehicle_simulator')
    skidpad_pkg_dir = get_package_share_directory('skidpad_planner')
    profiler_pkg_dir = get_package_share_directory('velocity_profiler')
    mpc_pkg_dir = get_package_share_directory('mpc_controller')
    viz_pkg_dir = get_package_share_directory('fsac_viz')
    huat_launch_dir = get_package_share_directory('huat_launch')
    benchmark_pkg_dir = get_package_share_directory('track_benchmark')

    default_track = os.path.join(sim_pkg_dir, 'tracks', 'skidpad_track.csv')
    default_sim_params = os.path.join(sim_pkg_dir, 'config', 'simulator_params.yaml')
    default_skidpad_params = os.path.join(skidpad_pkg_dir, 'config', 'skidpad_planner.yaml')
    default_profiler_params = os.path.join(profiler_pkg_dir, 'config', 'velocity_profiler_params.yaml')
    default_mpc_params = os.path.join(mpc_pkg_dir, 'config', 'mpc_params.yaml')
    default_viz_params = os.path.join(viz_pkg_dir, 'config', 'fsd_viz.yaml')
    default_rviz_config = os.path.join(huat_launch_dir, 'config', 'tuxiang.rviz')

    # 1. 车辆与传感器闭环仿真器
    sim_node = Node(
        package='vehicle_simulator',
        executable='vehicle_simulator_node',
        name='vehicle_simulator',
        output='screen',
        parameters=[
            default_sim_params,
            {
                'track_file': default_track,
                'init_x': -12.0,
                'init_y': 0.0,
                'init_theta': 0.0,
                'init_v': 0.0,
            }
        ]
    )

    # 2. 八字几何规划器 (输出未经过速度优化的 raw_pathlimits)
    skidpad_node = Node(
        package='skidpad_planner',
        executable='skidpad_planner',
        name='skidpad_planner',
        output='screen',
        parameters=[
            default_skidpad_params,
            {
                'road_type': 1,
                'input_cone_map_topic': '/sensors/cones/fused',
                'output_pathlimits_topic': '/planning/raw_pathlimits',
                'vehicle_state_topic': '/localization/vehicle_state',
            }
        ]
    )

    # 3. 极限速度剖面规划器 (Optimal Velocity Profiler)
    profiler_node = Node(
        package='velocity_profiler',
        executable='velocity_profiler_node',
        name='velocity_profiler_node',
        output='screen',
        parameters=[
            default_profiler_params,
            {
                'limits.max_velocity': 18.0,
                'limits.min_velocity': 3.0,
                'limits.max_lat_accel': 9.8,
                'limits.max_lon_accel': 4.0,
                'limits.max_lon_decel': 5.0,
                'topics.input_path': '/planning/raw_pathlimits',
                'topics.output_path': '/planning/pathlimits',
                'topics.vehicle_state': '/localization/vehicle_state',
                'topics.speed_markers': '/planning/viz/speed_markers',
            }
        ]
    )

    # 4. 高阶模型预测控制器 (读取带有速度约束的 pathlimits)
    mpc_node = Node(
        package='mpc_controller',
        executable='mpc_controller_node',
        name='mpc_controller_node',
        output='screen',
        parameters=[
            default_mpc_params,
            {
                'topics.vehicle_state': '/localization/vehicle_state',
                'topics.path': '/planning/pathlimits',
                'topics.vehicle_command': '/vehicle_command',
                'topics.stop': '/system/stop',
            }
        ]
    )

    # 5. 实时 KPI 基准性能评测看板
    benchmark_node = Node(
        package='track_benchmark',
        executable='track_benchmark_node',
        name='track_benchmark_node',
        output='screen',
        parameters=[{
            'track_type': 'skidpad',
            'controller_name': 'MPC+OptimalVelocity',
            'report_file': 'benchmark_skidpad_mpc_optimal.md'
        }]
    )

    # 6. 统一 RViz 可视化器
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

    # 7. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config] if os.path.exists(default_rviz_config) else []
    )

    return LaunchDescription([
        sim_node,
        skidpad_node,
        profiler_node,
        mpc_node,
        benchmark_node,
        viz_node,
        rviz_node
    ])
