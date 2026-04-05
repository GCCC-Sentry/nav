import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('pursuit_decision')
    params_file = os.path.join(pkg_dir, 'config', 'pursuit_params.yaml')

    # 使用 CycloneDDS 解决宿主机(Iron/Jazzy) 与 Docker(Humble) 的跨版本DDS通信问题
    # FastDDS 的 TypeLookupService 在不同 ROS 2 版本之间不兼容
    # CycloneDDS 不依赖 TypeHash, 可跨版本正常通信
    pursuit_node = Node(
        package='pursuit_decision',
        executable='pursuit_node',
        name='pursuit_decision',
        output='screen',
        parameters=[params_file],
        remappings=[
            # 自瞄发来的目标信息
            ('auto_aim_target_pos', 'auto_aim_target_pos'),
            # 导航代价地图
            ('global_costmap/costmap', 'global_costmap/costmap'),
            # 发出的导航目标
            ('goal_pose', 'goal_pose'),
            # 追击状态
            ('pursuit_status', 'pursuit_status'),
        ],
        additional_env={'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'},
    )

    return LaunchDescription([pursuit_node])
