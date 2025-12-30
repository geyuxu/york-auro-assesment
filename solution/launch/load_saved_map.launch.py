#!/usr/bin/env python3
"""
加载已保存地图的 Launch 文件
用于在 Gazebo + RViz 中显示之前保存的地图
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 参数声明
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='/workspaces/AURO2025/solution/saved_maps/map_latest.yaml',
        description='Path to saved map YAML file'
    )

    declare_robot_ns = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Robot namespace'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless'
    )

    declare_random_seed = DeclareLaunchArgument(
        'random_seed',
        default_value='42',
        description='Random seed for environment'
    )

    # 启动 Assessment 仿真环境
    assessment_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('assessment'),
                'launch',
                'assessment.launch.py'
            )
        ),
        launch_arguments={
            'num_robots': '1',
            'use_nav2': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'headless': LaunchConfiguration('headless'),
            'random_seed': LaunchConfiguration('random_seed'),
        }.items()
    )

    # 地图服务器节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=LaunchConfiguration('robot_namespace'),
        output='screen',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map_file')},
            {'use_sim_time': True},
        ]
    )

    # 地图发布器（lifecycle 管理）
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace=LaunchConfiguration('robot_namespace'),
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    return LaunchDescription([
        # 参数声明
        declare_map_file,
        declare_robot_ns,
        declare_use_rviz,
        declare_headless,
        declare_random_seed,

        # 节点
        assessment_launch,
        map_server_node,
        lifecycle_manager_node,
    ])
