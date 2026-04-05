# PB2025 哨兵追击节点 (Sentry Pursuit Node)

## 功能概述

对接 SP 视觉系统 (`sp_vision_25`) 的 ROS2 接口，实现哨兵机器人自动追击功能。

### 核心功能

| 功能 | 描述 |
|------|------|
| **自动追击** | 订阅 SP 视觉目标位置话题，自动计算并保持 3m 最佳攻击距离 |
| **禁区规避** | 支持手动划定矩形/多边形禁区，目标进入禁区时停止追击 |
| **车辆过滤** | 可配置不追击的车辆 ID 列表 (如前哨站、基地等固定目标) |
| **动态使能** | 通过 ROS2 话题动态启用/禁用追击模式 |
| **可视化** | RViz 中显示目标位置、攻击距离环、禁区高亮 |

## 系统架构

```
SP视觉 (sp_vision_25)                    追击节点                        Nav2
┌──────────────────┐            ┌──────────────────────┐        ┌────────────────┐
│                  │  String    │                      │ Action │                │
│ auto_aim_target  ├───────────►│  坐标变换 (TF2)      │───────►│ NavigateToPose │
│ _pos_publisher   │            │  禁区判断            │        │                │
│                  │            │  距离控制 (最佳3m)    │        └────────────────┘
│ "x,y,valid,id"  │            │  车辆过滤            │
└──────────────────┘            │  导航目标生成         │
                                └──────────────────────┘
```

## ROS2 接口

### 订阅话题

| 话题名 | 消息类型 | 描述 |
|--------|---------|------|
| `auto_aim_target_pos` | `std_msgs/String` | SP 视觉目标位置 (格式: `"x,y,valid,id"`) |
| `odom` | `nav_msgs/Odometry` | 机器人里程计 (TF 备用方案) |
| `pursuit/enable` | `std_msgs/Bool` | 追击使能控制 |

### 发布话题

| 话题名 | 消息类型 | 描述 |
|--------|---------|------|
| `pursuit/status` | `std_msgs/String` | 追击状态信息 |
| `pursuit/markers` | `visualization_msgs/MarkerArray` | RViz 可视化标记 |
| `pursuit/current_goal` | `geometry_msgs/PoseStamped` | 当前导航目标 |

### Action 客户端

| Action | 类型 | 描述 |
|--------|------|------|
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 导航目标 |

## 安装与构建

```bash
cd ~/tongji/sp20260313_kyb/nav_20260401_mppi_yes
colcon build --packages-select pb2025_sentry_pursuit
source install/setup.bash
```

## 使用方式

### 1. 启动节点

```bash
# 使用默认配置
ros2 launch pb2025_sentry_pursuit pursuit_launch.py

# 自定义攻击距离
ros2 launch pb2025_sentry_pursuit pursuit_launch.py optimal_attack_range:=2.5

# 初始禁用追击 (后续通过话题启用)
ros2 launch pb2025_sentry_pursuit pursuit_launch.py pursuit_enabled:=false
```

### 2. 动态控制

```bash
# 启用追击
ros2 topic pub --once /pursuit/enable std_msgs/msg/Bool "data: true"

# 禁用追击
ros2 topic pub --once /pursuit/enable std_msgs/msg/Bool "data: false"

# 检查追击状态
ros2 topic echo /pursuit/status
```

### 3. 状态码说明

| 状态 | 含义 |
|------|------|
| `DISABLED` | 追击功能已禁用 |
| `NO_TARGET` | 无目标 |
| `TARGET_LOST` | 目标丢失 |
| `NO_ROBOT_POSE` | 无法获取机器人位姿 |
| `TARGET_IN_FORBIDDEN:区域名` | 目标在禁区内 |
| `GOAL_IN_FORBIDDEN:区域名` | 导航目标在禁区内 |
| `IN_RANGE:距离:车辆名` | 已在攻击范围内 |
| `PURSUING:车辆名:距离` | 正在追击 |

## 配置说明

配置文件位于 `config/pursuit_config.yaml`。

### 车辆 ID 对照表

| ID | 车辆 |
|----|------|
| 1 | Hero (英雄) |
| 2 | Engineer (工程) |
| 3 | Infantry3 (步兵3号) |
| 4 | Infantry4 (步兵4号) |
| 5 | Infantry5 (步兵5号) |
| 6 | Outpost (前哨站) |
| 7 | Sentry (哨兵) |
| 8 | Base (基地) |

### 禁区配置示例

在 `pursuit_config.yaml` 中添加禁区:

```yaml
# 1. 在名称列表中添加
forbidden_zones_names:
  - "our_base"
  - "supply_area"
  - "custom_zone"

# 2. 定义顶点坐标 (地图坐标系, x1,y1,x2,y2,...)
zone_custom_zone:
  - 3.0    # 顶点1 x
  - -2.0   # 顶点1 y
  - 5.0    # 顶点2 x
  - -2.0   # 顶点2 y
  - 5.0    # 顶点3 x
  - -1.0   # 顶点3 y
  - 3.0    # 顶点4 x
  - -1.0   # 顶点4 y
```

### 不追击车辆配置

```yaml
# 不追击前哨站(6)和基地(8)
excluded_vehicles: [6, 8]

# 不追击英雄(1)和工程(2)
excluded_vehicles: [1, 2]

# 追击所有车辆
excluded_vehicles: []
```

## 坐标系说明

追击节点支持两种坐标变换方式:

1. **TF2 变换** (推荐): 通过 TF 树从 `gimbal_link` 变换到 `map`
2. **备用方案**: 当 TF 不可用时，基于 odom + 简化模型估算 (假设云台朝向与底盘一致)

如果你的 TF 树中云台坐标系名称不同，请修改 `gimbal_frame` 参数。

## 文件结构

```
pb2025_sentry_pursuit/
├── package.xml                              # ROS2 包描述
├── setup.py                                 # Python 构建配置
├── setup.cfg                                # 安装配置
├── resource/pb2025_sentry_pursuit           # ament 索引资源
├── config/
│   └── pursuit_config.yaml                  # 追击参数配置
├── launch/
│   └── pursuit_launch.py                    # Launch 文件
└── pb2025_sentry_pursuit/
    ├── __init__.py
    └── pursuit_node.py                      # 追击主节点
```
