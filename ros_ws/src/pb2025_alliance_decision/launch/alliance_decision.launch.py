#!/usr/bin/env python3
"""Launch 联盟赛哨兵决策节点"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pb2025_alliance_decision',
            executable='alliance_decision_node',
            name='alliance_decision_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'hp_supply_threshold': 120,    # HP 低于此值前往补给
                'hp_full': 400,                # 满血值
                'supply_timeout_sec': 5.0,     # 补给点最多待 5 秒
                'patrol_interval_sec': 5.0,    # 巡逻点切换间隔
                'spin_speed': 7.0,             # 小陀螺转速 rad/s
                'loop_rate_hz': 10.0,          # 主循环频率
            }],
        ),
    ])
