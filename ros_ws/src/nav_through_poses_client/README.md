# Nav Through Poses Client

这个功能包实现了通过 RViz2 选点并调用 nav2 的 NavigateThroughPoses 功能。

## 功能

- 在 RViz2 中点击多个目标点
- 自动收集所有点位
- 触发后调用 nav2 进行路径规划和导航

## 使用方法

### 1. 启动节点

```bash
# 在容器内
source /root/ros_ws/install/setup.bash
ros2 run nav_through_poses_client nav_through_poses_node
```

或使用 launch 文件：

```bash
ros2 launch nav_through_poses_client nav_through_poses_launch.py
```

### 2. 在 RViz2 中选点

1. 打开 RViz2
2. 添加 "Publish Point" 工具（如果没有的话）
3. 使用 "Publish Point" 工具在地图上点击多个目标点
4. 每次点击都会被记录

### 3. 触发导航

在另一个终端中发送触发命令：

```bash
ros2 topic pub --once /nav_trigger std_msgs/msg/String "data: 'start'"
```

## 话题说明

- **订阅话题**:
  - `/clicked_point` (geometry_msgs/PointStamped): 从 RViz2 接收点击的点
  - `/nav_trigger` (std_msgs/String): 触发导航的命令

- **Action Client**:
  - `navigate_through_poses` (nav2_msgs/action/NavigateThroughPoses): 调用 nav2 路径规划

## 注意事项

1. 确保 nav2 已经启动并运行
2. 确保 `navigate_through_poses` action server 可用
3. 点击的点会一直累积，直到触发导航后才会清空
