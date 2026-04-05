#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PB2025 哨兵追击节点 Launch 文件

用法:
  # 使用默认配置
  ros2 launch pb2025_sentry_pursuit pursuit_launch.py

  # 覆盖参数
  ros2 launch pb2025_sentry_pursuit pursuit_launch.py optimal_attack_range:=2.5

  # 禁用追击启动 (之后通过话题使能)
  ros2 launch pb2025_sentry_pursuit pursuit_launch.py pursuit_enabled:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('pb2025_sentry_pursuit')
    config_file = os.path.join(pkg_dir, 'config', 'pursuit_config.yaml')

    return LaunchDescription([
        # ==================== Launch 参数 ====================
        DeclareLaunchArgument(
            'pursuit_config',
            default_value=config_file,
            description='追击节点配置文件路径'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间'),

        DeclareLaunchArgument(
            'pursuit_enabled',
            default_value='true',
            description='初始是否启用追击'),

        DeclareLaunchArgument(
            'optimal_attack_range',
            default_value='3.0',
            description='最佳攻击距离 (m)'),

        # ==================== 追击节点 ====================
        Node(
            package='pb2025_sentry_pursuit',
            executable='pursuit_node',
            name='sentry_pursuit_node',
            output='screen',
            parameters=[
                LaunchConfiguration('pursuit_config'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                # 话题重映射 (根据实际系统调整)
                # ('auto_aim_target_pos', '/sp_vision/auto_aim_target_pos'),
                # ('odom', '/odom'),
            ],
        ),
    ])
