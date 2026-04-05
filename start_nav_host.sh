#!/bin/bash
# 在宿主机上运行此脚本，会打开多个独立终端窗口

CONTAINER="2026rmul"

# 检查容器是否运行
if ! docker ps | grep -q $CONTAINER; then
    echo "错误：容器 $CONTAINER 未运行"
    echo "请先启动容器：docker start $CONTAINER"
    exit 1
fi

echo "正在启动 ROS2 导航系统..."
echo "将打开 5 个独立终端窗口"



# 窗口1：rm_static_tf
gnome-terminal --title="rm_static_tf" -- bash -c "docker exec -it $CONTAINER bash -c 'cd /root/ros_ws && source install/setup.bash && ros2 launch rm_static_tf static_tf.launch.py'; exec bash" &

sleep 0.5

# 窗口2：adam_map2odom
gnome-terminal --title="adam_map2odom" -- bash -c "docker exec -it $CONTAINER bash -c 'cd /root/ros_ws && source install/setup.bash && ros2 launch adam_map2odom map2odom.launch.py'; exec bash" &

sleep 2

# 窗口3：pb2025_nav_bringup
gnome-terminal --title="pb2025_nav_bringup" -- bash -c "docker exec -it $CONTAINER bash -c 'cd /root/ros_ws && source install/setup.bash && ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=qian1 slam:=False use_robot_state_pub:=False'; exec bash" &

sleep 3

# 窗口4：my_serial_py
gnome-terminal --title="my_serial_py" -- bash -c "docker exec -it $CONTAINER bash -c 'cd /root/ros_ws && source install/setup.bash && ros2 launch my_serial_py serial.launch.py'; exec bash" &

sleep 3

# 窗口5：pb2025_sentry_behavior
gnome-terminal --title="pb2025_alliance_decision" -- bash -c "docker exec -it $CONTAINER bash -c 'cd /root/ros_ws && source install/setup.bash && ros2 launch pb2025_alliance_decision alliance_decision.launch.py '; exec bash" &

echo "======================================"
echo "所有终端窗口已启动！"
echo "======================================"
echo "每个 ROS 节点在独立的终端窗口中运行"
echo "你可以在每个窗口中独立滚动查看历史输出"
echo ""
echo "停止方法："
echo "  1. 在每个终端窗口中按 Ctrl+C"
echo "  2. 或关闭终端窗口"
echo "======================================"
