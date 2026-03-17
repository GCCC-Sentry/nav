#!/bin/bash
# 进入ROS2工作空间
cd ~/ros_ws

# 屏蔽 accessibility bus 警告
export NO_AT_BRIDGE=1

# 创建新的 tmux 会话
SESSION="ros_nav"

# 如果会话已存在，先关闭
tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION

# 创建新会话并运行第一个节点
tmux new-session -d -s $SESSION -n "rm_static_tf"
tmux send-keys -t $SESSION:0 "cd ~/ros_ws && source install/setup.bash && ros2 launch rm_static_tf static_tf.launch.py" C-m

# 创建第二个窗口
tmux new-window -t $SESSION:1 -n "adam_map2odom"
tmux send-keys -t $SESSION:1 "cd ~/ros_ws && source install/setup.bash && ros2 launch adam_map2odom map2odom.launch.py" C-m

# 等待2秒
sleep 2

# 创建第三个窗口
tmux new-window -t $SESSION:2 -n "pb2025_nav_bringup"
tmux send-keys -t $SESSION:2 "cd ~/ros_ws && source install/setup.bash && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=bl slam:=False use_robot_state_pub:=False" C-m

# 等待3秒
sleep 3

# 创建第四个窗口
tmux new-window -t $SESSION:3 -n "my_serial_py"
tmux send-keys -t $SESSION:3 "cd ~/ros_ws && source install/setup.bash && ros2 launch my_serial_py serial.launch.py" C-m

# 等待6秒
sleep 6

# 创建第五个窗口
tmux new-window -t $SESSION:4 -n "pb2025_sentry_behavior"
tmux send-keys -t $SESSION:4 "cd ~/ros_ws && source install/setup.bash && ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py" C-m

# 附加到会话
echo "======================================"
echo "ROS2 导航系统已在 tmux 会话中启动"
echo "======================================"
echo "tmux 快捷键："
echo "  Ctrl+b d     - 分离会话（后台运行）"
echo "  Ctrl+b 0-4   - 切换到对应窗口"
echo "  Ctrl+b n     - 下一个窗口"
echo "  Ctrl+b p     - 上一个窗口"
echo "  Ctrl+b w     - 列出所有窗口"
echo ""
echo "重新连接: tmux attach -t $SESSION"
echo "关闭所有: tmux kill-session -t $SESSION"
echo "======================================"

tmux attach -t $SESSION
