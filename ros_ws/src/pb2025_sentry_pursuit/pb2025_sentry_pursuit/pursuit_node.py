#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PB2025 哨兵追击节点 (Sentry Pursuit Node)

功能:
  1. 对接 SP 视觉系统的 auto_aim_target_pos 话题，获取目标位置与ID
  2. 通过 TF2 将目标从云台坐标系转换至地图坐标系
  3. 计算追击导航点，保持 3m 最佳攻击距离
  4. 支持手动划定禁区 (矩形/多边形)，目标进入禁区时停止追击
  5. 支持配置不追击的车辆ID列表
  6. 通过 Nav2 NavigateToPose Action 发送导航目标

SP 视觉接口:
  - 订阅: auto_aim_target_pos (std_msgs/String)  格式: "x,y,valid_flag,armor_id"
    - x, y: 目标在云台坐标系下的位置 (x=前方深度, y=左右偏移)
    - valid_flag: 1=有效目标
    - armor_id: 装甲板ID (1=hero, 2=engineer, 3=infantry3, 4=infantry4, 5=infantry5, 6=outpost, 7=sentry, 8=base)
  - 发布: enemy_status (sp_msgs/msg/EnemyStatusMsg) [可选, 通过nav系统]
"""

import math
import numpy as np
from enum import IntEnum
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point, PointStamped, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

import tf2_ros
import tf2_geometry_msgs


# ======================== 装甲板 ID 枚举 ========================
class ArmorID(IntEnum):
    """
    装甲板 ID 定义 (与 SP 视觉通信协议对应, armor.name + 1)
    """
    HERO = 1
    ENGINEER = 2
    INFANTRY_3 = 3
    INFANTRY_4 = 4
    INFANTRY_5 = 5
    OUTPOST = 6
    SENTRY = 7
    BASE = 8

    @classmethod
    def name_str(cls, val: int) -> str:
        _MAP = {
            1: "Hero", 2: "Engineer", 3: "Infantry3",
            4: "Infantry4", 5: "Infantry5", 6: "Outpost",
            7: "Sentry", 8: "Base"
        }
        return _MAP.get(val, f"Unknown({val})")


# ======================== 数据结构 ========================
@dataclass
class ForbiddenZone:
    """
    禁区定义 (凸多边形)
    vertices: 多边形顶点列表 [(x1,y1), (x2,y2), ...]
    name: 禁区名称 (用于日志)
    """
    vertices: List[Tuple[float, float]]
    name: str = "unnamed"

    def contains(self, px: float, py: float) -> bool:
        """射线法判断点是否在多边形内部"""
        n = len(self.vertices)
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = self.vertices[i]
            xj, yj = self.vertices[j]
            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside


@dataclass
class TargetState:
    """追踪目标的状态"""
    armor_id: int = 0
    map_x: float = 0.0
    map_y: float = 0.0
    last_seen_time: float = 0.0
    confidence: float = 0.0
    # 用于简单的位置平滑 (指数移动平均)
    ema_x: float = 0.0
    ema_y: float = 0.0
    initialized: bool = False


# ======================== 追击节点 ========================
class PursuitNode(Node):
    """
    哨兵追击决策节点

    通过订阅 SP 视觉的目标位置话题，结合 TF 坐标变换，
    计算追击导航目标点并通过 Nav2 执行导航。
    """

    def __init__(self):
        super().__init__('sentry_pursuit_node')

        # ==================== 参数声明 ====================
        self._declare_parameters()
        self._load_parameters()

        # ==================== TF2 ====================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ==================== 状态变量 ====================
        self.target_state = TargetState()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_valid = False
        self.pursuit_active = False
        self.current_goal_handle = None
        self.last_goal_time = 0.0
        self.nav_goal_active = False

        # ==================== 调试统计 ====================
        self._dbg_msg_count = 0          # 收到的总消息数
        self._dbg_valid_count = 0        # 有效目标数
        self._dbg_invalid_count = 0      # 无效目标数
        self._dbg_excluded_count = 0     # 被过滤的车辆数
        self._dbg_tf_ok_count = 0        # TF变换成功数
        self._dbg_tf_fail_count = 0      # TF变换失败数
        self._dbg_goal_sent_count = 0    # 发送导航目标数
        self._dbg_last_raw_msg = ''      # 最后一条原始消息
        self._dbg_last_summary_time = 0.0

        # ==================== 通信接口 ====================
        cb_group = ReentrantCallbackGroup()

        # 订阅 SP 视觉目标位置
        self.sub_target = self.create_subscription(
            String, 'auto_aim_target_pos',
            self.target_pos_callback, 10, callback_group=cb_group)

        # 订阅机器人位置 (通过 odom 或 amcl_pose)
        self.sub_odom = self.create_subscription(
            Odometry, 'odom',
            self.odom_callback, 10, callback_group=cb_group)

        # 追击使能控制 (外部可发送 true/false 启停追击)
        self.sub_enable = self.create_subscription(
            Bool, 'pursuit/enable',
            self.enable_callback, 10, callback_group=cb_group)

        # 发布追击状态 (供策略层使用)
        self.pub_status = self.create_publisher(
            String, 'pursuit/status', 10)

        # 发布追击目标可视化 (RViz)
        self.pub_markers = self.create_publisher(
            MarkerArray, 'pursuit/markers', 10)

        # 发布当前追击导航目标
        self.pub_goal_viz = self.create_publisher(
            PoseStamped, 'pursuit/current_goal', 10)

        # Nav2 NavigateToPose Action Client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=MutuallyExclusiveCallbackGroup())

        # ==================== 定时器 ====================
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop, callback_group=cb_group)

        self.viz_timer = self.create_timer(
            0.5, self.publish_visualization, callback_group=cb_group)

        # 调试信息定时打印 (每2秒输出一次汇总)
        self.debug_timer = self.create_timer(
            2.0, self._debug_summary_callback, callback_group=cb_group)

        self.get_logger().info('='*50)
        self.get_logger().info('  哨兵追击节点已启动')
        self.get_logger().info(f'  最佳攻击距离: {self.optimal_range:.1f}m')
        self.get_logger().info(f'  攻击范围: [{self.min_range:.1f}, {self.max_range:.1f}]m')
        self.get_logger().info(f'  禁区数量: {len(self.forbidden_zones)}')
        self.get_logger().info(f'  不追击车辆: {[ArmorID.name_str(v) for v in self.excluded_vehicles]}')
        self.get_logger().info(f'  云台坐标系: {self.gimbal_frame}')
        self.get_logger().info(f'  地图坐标系: {self.map_frame}')
        self.get_logger().info('='*50)

    # ==================== 参数管理 ====================
    def _declare_parameters(self):
        """声明所有 ROS2 参数"""
        # 追击距离参数
        self.declare_parameter('optimal_attack_range', 3.0)
        self.declare_parameter('min_attack_range', 1.5)
        self.declare_parameter('max_attack_range', 5.0)

        # 控制参数
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('goal_update_min_interval', 0.5)
        self.declare_parameter('target_lost_timeout', 2.0)
        self.declare_parameter('position_ema_alpha', 0.3)
        self.declare_parameter('goal_reach_tolerance', 0.3)

        # 坐标系参数
        self.declare_parameter('gimbal_frame', 'gimbal_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # 追击使能
        self.declare_parameter('pursuit_enabled', True)

        # 不追击的车辆ID列表 (对应 ArmorID 枚举值)
        # 例: [6, 8] 表示不追前哨站和基地
        self.declare_parameter('excluded_vehicles', [6, 8])

        # 禁区配置: 每个禁区是一个 YAML 列表,包含名称和顶点坐标
        # 在 YAML 中的格式示例:
        # forbidden_zones:
        #   - name: "our_base"
        #     vertices: [0.0, 0.0,  1.0, 0.0,  1.0, 1.0,  0.0, 1.0]
        self.declare_parameter('forbidden_zones_names', ['our_base', 'supply_area'])
        self.declare_parameter('forbidden_zones_vertices', [
            # our_base: 左下(0,-4.5) 右下(1.5,-4.5) 右上(1.5,-3.0) 左上(0,-3.0)
            # supply_area: (7.0,-4.5) (8.5,-4.5) (8.5,-3.0) (7.0,-3.0)
        ])
        # 使用单独参数定义每个禁区的顶点(更灵活)
        self.declare_parameter('zone_our_base',
            [0.0, -4.5, 1.5, -4.5, 1.5, -3.0, 0.0, -3.0])
        self.declare_parameter('zone_supply_area',
            [7.0, -4.5, 8.5, -4.5, 8.5, -3.0, 7.0, -3.0])

    def _load_parameters(self):
        """加载参数到实例变量"""
        self.optimal_range = self.get_parameter('optimal_attack_range').value
        self.min_range = self.get_parameter('min_attack_range').value
        self.max_range = self.get_parameter('max_attack_range').value

        self.control_rate = self.get_parameter('control_rate').value
        self.goal_update_interval = self.get_parameter('goal_update_min_interval').value
        self.target_lost_timeout = self.get_parameter('target_lost_timeout').value
        self.ema_alpha = self.get_parameter('position_ema_alpha').value
        self.goal_tolerance = self.get_parameter('goal_reach_tolerance').value

        self.gimbal_frame = self.get_parameter('gimbal_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.pursuit_enabled = self.get_parameter('pursuit_enabled').value
        self.excluded_vehicles = list(self.get_parameter('excluded_vehicles').value)

        # 加载禁区
        self.forbidden_zones: List[ForbiddenZone] = []
        zone_names = self.get_parameter('forbidden_zones_names').value
        for zname in zone_names:
            param_name = f'zone_{zname}'
            try:
                verts_flat = self.get_parameter(param_name).value
                if len(verts_flat) >= 6 and len(verts_flat) % 2 == 0:
                    vertices = [(verts_flat[i], verts_flat[i+1])
                                for i in range(0, len(verts_flat), 2)]
                    self.forbidden_zones.append(ForbiddenZone(vertices=vertices, name=zname))
                    self.get_logger().info(
                        f'加载禁区 [{zname}]: {len(vertices)} 个顶点')
            except rclpy.exceptions.ParameterNotDeclaredException:
                self.get_logger().warn(f'禁区参数 {param_name} 未定义，跳过')

    # ==================== 调试汇总 ====================
    def _debug_summary_callback(self):
        """定时打印调试汇总信息"""
        now = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('  📊 [追击节点调试汇总]')
        self.get_logger().info(f'  收到消息总数: {self._dbg_msg_count} | '
                               f'有效: {self._dbg_valid_count} | '
                               f'无效: {self._dbg_invalid_count} | '
                               f'过滤: {self._dbg_excluded_count}')
        self.get_logger().info(f'  TF变换 成功: {self._dbg_tf_ok_count} | '
                               f'失败: {self._dbg_tf_fail_count}')
        self.get_logger().info(f'  导航目标已发送: {self._dbg_goal_sent_count} | '
                               f'导航活跃: {self.nav_goal_active}')
        self.get_logger().info(f'  追击使能: {self.pursuit_enabled} | '
                               f'追击活跃: {self.pursuit_active}')
        self.get_logger().info(f'  机器人位姿有效: {self.robot_pose_valid} | '
                               f'位置: ({self.robot_x:.2f}, {self.robot_y:.2f}) yaw={math.degrees(self.robot_yaw):.1f}°')
        if self.target_state.initialized:
            dt = now - self.target_state.last_seen_time
            dist = math.hypot(self.target_state.map_x - self.robot_x,
                              self.target_state.map_y - self.robot_y)
            self.get_logger().info(
                f'  目标: {ArmorID.name_str(self.target_state.armor_id)} '
                f'@ map({self.target_state.map_x:.2f}, {self.target_state.map_y:.2f}) | '
                f'置信度: {self.target_state.confidence:.2f} | '
                f'上次看到: {dt:.1f}s前 | '
                f'距离: {dist:.2f}m')
        else:
            self.get_logger().info('  目标: 无 (未初始化)')
        self.get_logger().info(f'  最后原始消息: "{self._dbg_last_raw_msg}"')
        self.get_logger().info('='*60)

    # ==================== 回调函数 ====================
    def target_pos_callback(self, msg: String):
        """
        处理 SP 视觉目标位置话题回调

        消息格式: "x,y,valid,id"
        - x: 云台坐标系 x (前方深度, 单位: m)
        - y: 云台坐标系 y (左右偏移, 单位: m)
        - valid: 有效标志 (1=有效, 0=无效)
        - id: 装甲板 ID (1~8, 参见 ArmorID)
        """
        self._dbg_msg_count += 1
        self._dbg_last_raw_msg = msg.data

        try:
            parts = msg.data.split(',')
            if len(parts) < 4:
                self.get_logger().warn(
                    f'[识别] ⚠ 消息格式错误, 字段不足4个: "{msg.data}"')
                return

            gimbal_x = float(parts[0])
            gimbal_y = float(parts[1])
            valid = float(parts[2])
            armor_id = int(float(parts[3]))

            self.get_logger().info(
                f'[识别] 收到目标数据 | 云台坐标: ({gimbal_x:.3f}, {gimbal_y:.3f}) | '
                f'有效: {valid} | 装甲板ID: {armor_id}({ArmorID.name_str(armor_id)})')

            # 无效目标
            if valid < 0.5 or (gimbal_x == 0.0 and gimbal_y == 0.0):
                self._dbg_invalid_count += 1
                self.get_logger().info(
                    f'[识别] ✗ 目标无效 (valid={valid}, pos=({gimbal_x:.3f},{gimbal_y:.3f})), 跳过')
                return

            self._dbg_valid_count += 1

            # 过滤不追击的车辆
            if armor_id in self.excluded_vehicles:
                self._dbg_excluded_count += 1
                self.get_logger().info(
                    f'[识别] ✗ 目标 {ArmorID.name_str(armor_id)}(ID={armor_id}) '
                    f'在排除列表中, 不追击')
                return

            # 尝试将目标位置 (云台坐标系) 转换到地图坐标系
            self.get_logger().info(
                f'[坐标变换] 尝试 {self.gimbal_frame} -> {self.map_frame} ...'
                f' 云台坐标: ({gimbal_x:.3f}, {gimbal_y:.3f}, 0.0)')
            map_pos = self._transform_to_map(gimbal_x, gimbal_y, 0.0)
            tf_method = 'TF2'
            if map_pos is None:
                self.get_logger().warn(
                    f'[坐标变换] ✗ TF2变换失败, 尝试odom备用方案...')
                # TF 不可用时使用备用方案: 基于 odom 估算
                map_pos = self._estimate_map_position(gimbal_x, gimbal_y)
                tf_method = 'odom估算'
                if map_pos is None:
                    self._dbg_tf_fail_count += 1
                    self.get_logger().error(
                        f'[坐标变换] ✗✗ TF2和odom备用方案均失败! '
                        f'robot_pose_valid={self.robot_pose_valid}')
                    return

            self._dbg_tf_ok_count += 1
            target_x, target_y = map_pos
            self.get_logger().info(
                f'[坐标变换] ✓ 成功({tf_method}) | '
                f'云台({gimbal_x:.3f}, {gimbal_y:.3f}) -> 地图({target_x:.2f}, {target_y:.2f})')

            # 更新目标状态 (指数移动平均平滑)
            now = self.get_clock().now().nanoseconds / 1e9
            if not self.target_state.initialized or armor_id != self.target_state.armor_id:
                # 新目标或切换目标
                self.target_state = TargetState(
                    armor_id=armor_id,
                    map_x=target_x, map_y=target_y,
                    ema_x=target_x, ema_y=target_y,
                    last_seen_time=now,
                    confidence=1.0,
                    initialized=True
                )
                self.get_logger().info(
                    f'🎯 [目标更新] 新目标识别! {ArmorID.name_str(armor_id)} '
                    f'@ 地图({target_x:.2f}, {target_y:.2f})')
            else:
                alpha = self.ema_alpha
                old_ema_x, old_ema_y = self.target_state.ema_x, self.target_state.ema_y
                self.target_state.ema_x = alpha * target_x + (1 - alpha) * self.target_state.ema_x
                self.target_state.ema_y = alpha * target_y + (1 - alpha) * self.target_state.ema_y
                self.target_state.map_x = self.target_state.ema_x
                self.target_state.map_y = self.target_state.ema_y
                self.target_state.last_seen_time = now
                self.target_state.confidence = min(1.0, self.target_state.confidence + 0.1)
                self.get_logger().info(
                    f'[目标更新] EMA平滑 | '
                    f'原始({target_x:.2f},{target_y:.2f}) '
                    f'EMA({old_ema_x:.2f},{old_ema_y:.2f})->'
                    f'({self.target_state.ema_x:.2f},{self.target_state.ema_y:.2f}) | '
                    f'置信度: {self.target_state.confidence:.2f}')

        except (ValueError, IndexError) as e:
            self.get_logger().error(f'[识别] ✗ 解析目标数据异常: "{msg.data}" - {e}')

    def odom_callback(self, msg: Odometry):
        """更新机器人在 odom 坐标系下的位置"""
        prev_valid = self.robot_pose_valid
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_pose_valid = True
        if not prev_valid:
            self.get_logger().info(
                f'[里程计] ✓ 首次收到odom | '
                f'位置: ({self.robot_x:.2f}, {self.robot_y:.2f}) yaw={math.degrees(self.robot_yaw):.1f}°')

    def enable_callback(self, msg: Bool):
        """外部使能/禁用追击"""
        prev = self.pursuit_enabled
        self.pursuit_enabled = msg.data
        if prev != msg.data:
            self.get_logger().info(
                f'追击模式: {"启用" if msg.data else "禁用"}')
            if not msg.data:
                self._cancel_navigation()

    # ==================== 坐标变换 ====================
    def _transform_to_map(self, x: float, y: float, z: float) -> Optional[Tuple[float, float]]:
        """
        将云台坐标系下的点变换到地图坐标系

        SP 视觉 gimbal 坐标系: x=前方(深度), y=左(水平), z=上(垂直)
        """
        try:
            point = PointStamped()
            point.header.frame_id = self.gimbal_frame
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = x
            point.point.y = y
            point.point.z = z

            transformed = self.tf_buffer.transform(
                point, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.1))

            return (transformed.point.x, transformed.point.y)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'[坐标变换] TF异常 ({self.gimbal_frame}->{self.map_frame}): {e}')
            return None

    def _estimate_map_position(self, gimbal_x: float, gimbal_y: float) -> Optional[Tuple[float, float]]:
        """
        TF 不可用时的备用方案:
        基于 odom 位姿估算目标在地图坐标系的位置

        假设云台朝向与底盘朝向一致 (简化模型)
        """
        if not self.robot_pose_valid:
            return None

        # 云台坐标系: x=前(depth), y=左
        # 旋转到地图坐标系
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        map_x = self.robot_x + gimbal_x * cos_yaw - gimbal_y * sin_yaw
        map_y = self.robot_y + gimbal_x * sin_yaw + gimbal_y * cos_yaw

        return (map_x, map_y)

    def _get_robot_map_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        通过 TF 获取机器人在 map 坐标系下的位姿

        Returns: (x, y, yaw) 或 None
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return (x, y, yaw)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # fallback 到 odom
            if self.robot_pose_valid:
                return (self.robot_x, self.robot_y, self.robot_yaw)
            return None

    # ==================== 禁区判断 ====================
    def _is_in_forbidden_zone(self, x: float, y: float) -> Tuple[bool, str]:
        """
        判断坐标点是否在任何禁区内

        Returns: (是否在禁区内, 禁区名称)
        """
        for zone in self.forbidden_zones:
            if zone.contains(x, y):
                return True, zone.name
        return False, ""

    def _is_path_through_forbidden(self, x1: float, y1: float,
                                    x2: float, y2: float, samples: int = 10) -> Tuple[bool, str]:
        """
        检查从 (x1,y1) 到 (x2,y2) 的路径是否穿过禁区
        通过采样路径上的点来判断
        """
        for i in range(samples + 1):
            t = i / samples
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            in_zone, zone_name = self._is_in_forbidden_zone(px, py)
            if in_zone:
                return True, zone_name
        return False, ""

    # ==================== 追击逻辑 ====================
    def _compute_pursuit_goal(self, target_x: float, target_y: float,
                               robot_x: float, robot_y: float) -> Optional[Tuple[float, float, float]]:
        """
        计算追击导航目标点

        策略: 在目标与机器人连线上, 距目标 optimal_range 的位置
        如果已在 [min_range, max_range] 内, 则不更新目标

        Returns: (goal_x, goal_y, goal_yaw) 或 None (无需移动)
        """
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.hypot(dx, dy)

        if dist < 0.01:
            return None

        # 已经在最佳攻击范围内
        if self.min_range <= dist <= self.max_range:
            # 只需要朝向目标
            yaw = math.atan2(dy, dx)
            return None  # 不需要移动，距离合适

        # 计算追击点: 在连线上距目标 optimal_range 处
        # 方向: 从目标指向机器人
        unit_x = -dx / dist  # 目标到机器人的单位向量
        unit_y = -dy / dist

        goal_x = target_x + unit_x * self.optimal_range
        goal_y = target_y + unit_y * self.optimal_range

        # 朝向目标的 yaw
        goal_yaw = math.atan2(dy, dx)

        return (goal_x, goal_y, goal_yaw)

    def control_loop(self):
        """
        主控制循环 (定时回调)
        """
        now = self.get_clock().now().nanoseconds / 1e9

        # 发布状态信息
        status_msg = String()

        # ---- 前置检查 ----
        if not self.pursuit_enabled:
            status_msg.data = "DISABLED"
            self.pub_status.publish(status_msg)
            self.get_logger().debug('[控制循环] 追击未使能, 跳过')
            return

        if not self.target_state.initialized:
            status_msg.data = "NO_TARGET"
            self.pub_status.publish(status_msg)
            self.get_logger().debug('[控制循环] 无目标, 等待识别...')
            return

        # 目标丢失检测
        dt = now - self.target_state.last_seen_time
        if dt > self.target_lost_timeout:
            self.get_logger().warn(
                f'[控制循环] ⚠ 目标丢失! '
                f'{ArmorID.name_str(self.target_state.armor_id)} '
                f'已 {dt:.1f}s 未更新 (超时阈值: {self.target_lost_timeout:.1f}s), 停止追击')
            self._cancel_navigation()
            self.target_state.initialized = False
            self.pursuit_active = False
            status_msg.data = "TARGET_LOST"
            self.pub_status.publish(status_msg)
            return

        # 随时间衰减置信度
        self.target_state.confidence = max(0.0, self.target_state.confidence - 0.02)

        # 获取机器人位置
        robot_pose = self._get_robot_map_pose()
        if robot_pose is None:
            status_msg.data = "NO_ROBOT_POSE"
            self.pub_status.publish(status_msg)
            return

        robot_x, robot_y, robot_yaw = robot_pose
        target_x = self.target_state.map_x
        target_y = self.target_state.map_y

        dist_debug = math.hypot(target_x - robot_x, target_y - robot_y)
        self.get_logger().info(
            f'[控制循环] 目标: {ArmorID.name_str(self.target_state.armor_id)} '
            f'@ ({target_x:.2f},{target_y:.2f}) | '
            f'机器人: ({robot_x:.2f},{robot_y:.2f}) | '
            f'距离: {dist_debug:.2f}m | '
            f'范围: [{self.min_range:.1f}, {self.max_range:.1f}]m')

        # ---- 禁区检查: 目标在禁区内 → 不追击 ----
        in_zone, zone_name = self._is_in_forbidden_zone(target_x, target_y)
        if in_zone:
            self.get_logger().info(
                f'目标 {ArmorID.name_str(self.target_state.armor_id)} '
                f'位于禁区 [{zone_name}], 停止追击')
            self._cancel_navigation()
            self.pursuit_active = False
            status_msg.data = f"TARGET_IN_FORBIDDEN:{zone_name}"
            self.pub_status.publish(status_msg)
            return

        # ---- 计算追击目标点 ----
        pursuit_goal = self._compute_pursuit_goal(target_x, target_y, robot_x, robot_y)

        dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)

        if pursuit_goal is None:
            # 已在最佳攻击范围内
            self.pursuit_active = True
            self.get_logger().info(
                f'[追击] ✓ 已在最佳攻击范围内! 距离: {dist_to_target:.2f}m '
                f'(范围: [{self.min_range:.1f}, {self.max_range:.1f}]m), 无需移动')
            status_msg.data = f"IN_RANGE:{dist_to_target:.2f}m:{ArmorID.name_str(self.target_state.armor_id)}"
            self.pub_status.publish(status_msg)
            return

        goal_x, goal_y, goal_yaw = pursuit_goal
        self.get_logger().info(
            f'[追击] → 计算导航目标: ({goal_x:.2f}, {goal_y:.2f}) '
            f'yaw={math.degrees(goal_yaw):.1f}° | '
            f'当前距离: {dist_to_target:.2f}m -> 目标距离: {self.optimal_range:.1f}m')

        # ---- 禁区检查: 导航目标点在禁区内 → 调整到禁区边界外 ----
        goal_in_zone, goal_zone = self._is_in_forbidden_zone(goal_x, goal_y)
        if goal_in_zone:
            adjusted = self._adjust_goal_outside_forbidden(
                goal_x, goal_y, target_x, target_y, robot_x, robot_y)
            if adjusted is None:
                self.get_logger().info(
                    f'导航目标在禁区 [{goal_zone}] 内且无法调整, 停止追击')
                self._cancel_navigation()
                self.pursuit_active = False
                status_msg.data = f"GOAL_IN_FORBIDDEN:{goal_zone}"
                self.pub_status.publish(status_msg)
                return
            goal_x, goal_y = adjusted
            goal_yaw = math.atan2(target_y - goal_y, target_x - goal_x)

        # ---- 路径穿过禁区检查 ----
        path_blocked, blocked_zone = self._is_path_through_forbidden(
            robot_x, robot_y, goal_x, goal_y)
        if path_blocked:
            self.get_logger().debug(
                f'路径穿过禁区 [{blocked_zone}], Nav2 将自动绕行')
            # 注: 不阻止导航, Nav2 全局规划器会绕过 costmap 中的障碍

        # ---- 限制目标更新频率 ----
        time_since_last = now - self.last_goal_time
        if time_since_last < self.goal_update_interval:
            self.get_logger().debug(
                f'[追击] 目标更新节流: 距上次 {time_since_last:.2f}s < {self.goal_update_interval:.1f}s')
            return

        # ---- 发送导航目标 ----
        self.get_logger().info(
            f'[导航] >>> 发送导航目标! ({goal_x:.2f}, {goal_y:.2f}) '
            f'yaw={math.degrees(goal_yaw):.1f}°')
        self._send_navigation_goal(goal_x, goal_y, goal_yaw)
        self._dbg_goal_sent_count += 1
        self.pursuit_active = True
        self.last_goal_time = now

        status_msg.data = (
            f"PURSUING:{ArmorID.name_str(self.target_state.armor_id)}"
            f":{dist_to_target:.2f}m")
        self.pub_status.publish(status_msg)

        self.get_logger().info(
            f'追击 {ArmorID.name_str(self.target_state.armor_id)} | '
            f'目标: ({target_x:.2f},{target_y:.2f}) | '
            f'导航点: ({goal_x:.2f},{goal_y:.2f}) | '
            f'距离: {dist_to_target:.2f}m → {self.optimal_range:.1f}m')

    def _adjust_goal_outside_forbidden(self, goal_x: float, goal_y: float,
                                        target_x: float, target_y: float,
                                        robot_x: float, robot_y: float) -> Optional[Tuple[float, float]]:
        """
        当导航目标在禁区内时，尝试在禁区外找到替代点

        策略: 围绕目标在不同角度上搜索距离 optimal_range 的点，
        选择离原始目标最近且不在禁区内的点
        """
        best = None
        best_dist = float('inf')

        for deg in range(0, 360, 15):
            rad = math.radians(deg)
            cx = target_x + self.optimal_range * math.cos(rad)
            cy = target_y + self.optimal_range * math.sin(rad)

            in_zone, _ = self._is_in_forbidden_zone(cx, cy)
            if not in_zone:
                d = math.hypot(cx - goal_x, cy - goal_y)
                if d < best_dist:
                    best_dist = d
                    best = (cx, cy)

        return best

    # ==================== 导航控制 ====================
    def _send_navigation_goal(self, x: float, y: float, yaw: float):
        """通过 Nav2 NavigateToPose Action 发送导航目标"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('[导航] ✗ Nav2 action server 不可用! 无法发送目标')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # yaw 转四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)

        # 发布可视化
        self.pub_goal_viz.publish(goal_msg.pose)

        # 取消之前的导航目标
        if self.current_goal_handle is not None and self.nav_goal_active:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass

        # 发送新目标
        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_callback)
        send_future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        """导航目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[导航] ✗ 导航目标被Nav2拒绝!')
            self.nav_goal_active = False
            return

        self.get_logger().info('[导航] ✓ Nav2 已接受导航目标')
        self.current_goal_handle = goal_handle
        self.nav_goal_active = True

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        """导航反馈回调 (可用于实时距离监控)"""
        fb = feedback_msg.feedback
        if hasattr(fb, 'distance_remaining'):
            self.get_logger().info(
                f'[导航反馈] 剩余距离: {fb.distance_remaining:.2f}m')

    def _nav_result_callback(self, future):
        """导航完成回调"""
        self.nav_goal_active = False
        self.get_logger().info('[导航] 导航目标执行完成')

    def _cancel_navigation(self):
        """取消当前导航目标"""
        if self.current_goal_handle is not None and self.nav_goal_active:
            try:
                self.current_goal_handle.cancel_goal_async()
                self.get_logger().info('[导航] 已取消当前导航目标')
            except Exception as e:
                self.get_logger().warn(f'[导航] 取消导航目标异常: {e}')
            self.nav_goal_active = False
        else:
            self.get_logger().debug('[导航] 无活跃导航目标需要取消')

    # ==================== 可视化 ====================
    def publish_visualization(self):
        """发布 RViz 可视化标记"""
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1. 禁区可视化 (红色半透明多边形)
        for i, zone in enumerate(self.forbidden_zones):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.map_frame
            marker.ns = "forbidden_zones"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = 1.0
            marker.color.a = 0.7

            for vx, vy in zone.vertices:
                p = Point()
                p.x = vx
                p.y = vy
                p.z = 0.05
                marker.points.append(p)
            # 闭合多边形
            if zone.vertices:
                p = Point()
                p.x = zone.vertices[0][0]
                p.y = zone.vertices[0][1]
                p.z = 0.05
                marker.points.append(p)

            markers.markers.append(marker)

        # 2. 目标位置可视化 (绿色球)
        if self.target_state.initialized:
            target_marker = Marker()
            target_marker.header.stamp = stamp
            target_marker.header.frame_id = self.map_frame
            target_marker.ns = "target"
            target_marker.id = 100
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = self.target_state.map_x
            target_marker.pose.position.y = self.target_state.map_y
            target_marker.pose.position.z = 0.3
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            target_marker.color.g = 1.0
            target_marker.color.a = 0.8
            markers.markers.append(target_marker)

            # 目标文字标签
            text_marker = Marker()
            text_marker.header.stamp = stamp
            text_marker.header.frame_id = self.map_frame
            text_marker.ns = "target_text"
            text_marker.id = 101
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = self.target_state.map_x
            text_marker.pose.position.y = self.target_state.map_y
            text_marker.pose.position.z = 0.7
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = (
                f'{ArmorID.name_str(self.target_state.armor_id)} '
                f'conf:{self.target_state.confidence:.1f}')
            markers.markers.append(text_marker)

            # 3. 最佳攻击距离环 (蓝色圆圈)
            ring_marker = Marker()
            ring_marker.header.stamp = stamp
            ring_marker.header.frame_id = self.map_frame
            ring_marker.ns = "attack_range"
            ring_marker.id = 102
            ring_marker.type = Marker.LINE_STRIP
            ring_marker.action = Marker.ADD
            ring_marker.scale.x = 0.03
            ring_marker.color.b = 1.0
            ring_marker.color.a = 0.5

            for deg in range(0, 361, 10):
                rad = math.radians(deg)
                p = Point()
                p.x = self.target_state.map_x + self.optimal_range * math.cos(rad)
                p.y = self.target_state.map_y + self.optimal_range * math.sin(rad)
                p.z = 0.02
                ring_marker.points.append(p)
            markers.markers.append(ring_marker)

        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_navigation()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
