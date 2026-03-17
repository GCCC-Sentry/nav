#!/bin/bash
# 进入ROS2工作空间
cd ~/ros_ws

# 屏蔽 accessibility bus 警告
export NO_AT_BRIDGE=1

# 创建新的 tmux 会话
SESSION="ros_nav"

# 如果会话已存在，先关闭
tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION

# 创建新会话
tmux new-session -d -s $SESSION

# 第一个面板：rm_static_tf
tmux send-keys -t $SESSION "cd ~/ros_ws && source install/setup.bash && echo '=== rm_static_tf ===' && ros2 launch rm_static_tf static_tf.launch.py" C-m

# 水平分割：adam_map2odom
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd ~/ros_ws && source install/setup.bash && sleep 1 && echo '=== adam_map2odom ===' && ros2 launch adam_map2odom map2odom.launch.py" C-m

# 选择左侧面板，垂直分割：pb2025_nav_bringup
tmux select-pane -t 0
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "cd ~/ros_ws && source install/setup.bash && sleep 3 && echo '=== pb2025_nav_bringup ===' && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=bl slam:=False use_robot_state_pub:=False" C-m

# 选择右侧面板，垂直分割：my_serial_py
tmux select-pane -t 1
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "cd ~/ros_ws && source install/setup.bash && sleep 6 && echo '=== my_serial_py ===' && ros2 launch my_serial_py serial.launch.py" C-m

# 选择右下面板，再垂直分割：pb2025_sentry_behavior
tmux select-pane -t 3
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "cd ~/ros_ws && source install/setup.bash && sleep 9 && echo '=== pb2025_sentry_behavior ===' && ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py" C-m

# 调整布局使其更均匀
tmux select-layout -t $SESSION tiled

echo "======================================"
echo "ROS2 导航系统已启动（分屏模式）"
echo "======================================"
echo "所有节点输出将同时显示在不同面板中"
echo ""
echo "tmux 快捷键："
echo "  Ctrl+b 方向键  - 切换面板焦点"
echo "  Ctrl+b z       - 放大/还原当前面板"
echo "  Ctrl+b d       - 分离会话（后台运行）"
echo "  Ctrl+b [       - 进入滚动模式（q 退出）"
echo ""
echo "重新连接: tmux attach -t $SESSION"
echo "关闭所有: tmux kill-session -t $SESSION"
echo "======================================"

tmux attach -t $SESSION
