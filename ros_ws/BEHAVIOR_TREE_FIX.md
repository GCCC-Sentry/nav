# 行为树类型转换错误修复总结

## 问题描述
运行行为树时出现错误：
```
Behavior Tree exception: The port with name "goal" and value "0;0;0" can not be converted to geometry_msgs::msg::PoseStamped_<std::allocator<void> >
```

## 根本原因
`PubNav2Goal` 插件试图将字符串 "0;0;0" 转换为 `geometry_msgs::msg::PoseStamped` 类型，但 BehaviorTree.CPP 的类型转换系统在动态加载插件时无法找到自定义类型转换函数。

## 解决方案
将 `PubNav2Goal` 的实现方式改为使用三个独立的 `double` 参数（x, y, yaw），而不是一个需要自定义类型转换的 `PoseStamped` 字符串。这与 `SendNav2Goal` 插件的实现方式一致。

## 修改的文件

### 1. plugins/action/pub_nav2_goal.cpp
**修改前：**
- 使用 `BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "0;0;0", ...)`
- 依赖自定义类型转换函数 `BT::convertFromString<PoseStamped>`

**修改后：**
- 使用三个独立参数：
  - `BT::InputPort<double>("x", 0.0, "Goal X coordinate")`
  - `BT::InputPort<double>("y", 0.0, "Goal Y coordinate")`
  - `BT::InputPort<double>("yaw", 0.0, "Goal Yaw angle (rad)")`
- 在 `setMessage()` 中手动构建 `PoseStamped` 消息

### 2. behavior_trees/rmul_2026_patrol.xml
**修改前：**
```xml
<PubNav2Goal goal="0;0;0" topic_name="goal_pose"/>
```

**修改后：**
```xml
<PubNav2Goal x="0" y="0" yaw="0" topic_name="goal_pose"/>
```

同时更新了 TreeNodesModel 中的端口定义。

## 验证结果
修复后，行为树成功加载并运行，不再出现类型转换错误。

## 备份文件位置
- src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_sentry_behavior/src/pb2025_sentry_behavior_server.cpp.bak
- src/pb2025_sentry_nav/nav_adam_docker/src/pb2025_sentry_nav/pb2025_sentry_behavior/behavior_trees/rmul_2026_patrol.xml.bak

修复日期：2026-02-08
