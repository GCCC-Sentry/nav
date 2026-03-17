#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from nav2_msgs.action import FollowPath
from std_msgs.msg import Header
import math
import tkinter as tk
from threading import Thread
import numpy as np
from scipy.interpolate import splprep, splev

class PathDrawerGUI(Node):
    def __init__(self):
        super().__init__('path_drawer_gui')
        
        # 1. 配置参数
        self.declare_parameter('controller_id', 'FollowPath')
        self.controller_id = self.get_parameter('controller_id').value
        
        # 2. 订阅 RViz 点击
        self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)
        
        # 3. 发布话题
        self.path_pub = self.create_publisher(Path, '/drawn_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/clicked_points_marker', 10)
        
        # 4. Nav2 Action Client
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        
        self.raw_points = [] 
        self.final_path_msg = None
        self.get_logger().info("GUI Drawer Ready! 等待在 RViz 点击...")

    def click_callback(self, msg):
        self.raw_points.append([msg.point.x, msg.point.y])
        self.update_path_visualization()
        self.get_logger().info(f"添加点: {msg.point.x:.2f}, {msg.point.y:.2f}")

    def update_path_visualization(self):
        if not self.raw_points:
            return
            
        # 准备显示用的 Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # 简单的平滑处理
        display_points = []
        if len(self.raw_points) >= 3:
            try:
                pts = np.array(self.raw_points).T
                tck, u = splprep(pts, s=0.0, k=min(3, len(self.raw_points)-1))
                u_new = np.linspace(0, 1, num=len(self.raw_points)*10)
                x_new, y_new = splev(u_new, tck)
                display_points = list(zip(x_new, y_new))
            except Exception:
                display_points = self.raw_points
        else:
            display_points = self.raw_points

        # 填充 Path 消息
        for i, (x, y) in enumerate(display_points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            
            # 计算朝向
            if i < len(display_points) - 1:
                next_x, next_y = display_points[i+1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            elif i > 0:
                pose.pose.orientation = path_msg.poses[-1].pose.orientation
            else:
                pose.pose.orientation.w = 1.0
                
            path_msg.poses.append(pose)

        self.final_path_msg = path_msg
        self.path_pub.publish(path_msg)
        self.publish_markers()

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0
        
        for pt in self.raw_points:
            p = PointStamped().point
            p.x = float(pt[0]); p.y = float(pt[1])
            marker.points.append(p)
        self.marker_pub.publish(marker)

    def start_navigation(self):
        if not self.final_path_msg:
            print("请先画线！")
            return
        
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            print("Nav2 Action Server 未连接! 请检查 Nav2 是否启动")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = self.final_path_msg
        goal_msg.controller_id = self.controller_id

        print("正在发送轨迹给控制器...")
        self._action_client.send_goal_async(goal_msg)

    def clear_path(self):
        self.raw_points = []
        self.final_path_msg = None
        # 清除显示
        empty = Path()
        empty.header.frame_id = "map"
        self.path_pub.publish(empty)
        
        m = Marker()
        m.header.frame_id = "map"
        m.action = Marker.DELETEALL
        self.marker_pub.publish(m)
        print("已清除")

def gui_thread(node):
    root = tk.Tk()
    root.title("轨迹画板")
    root.geometry("200x150")
    root.attributes('-topmost', True) # 窗口置顶

    tk.Label(root, text="ROS 2 轨迹画板", font=('Arial', 10)).pack(pady=5)
    
    tk.Button(root, text="▶ 开始执行", bg="#90EE90", command=node.start_navigation, height=2).pack(fill='x', padx=10, pady=5)
    tk.Button(root, text="🗑 清除重画", bg="#FFB6C1", command=node.clear_path, height=2).pack(fill='x', padx=10, pady=5)

    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = PathDrawerGUI()
    
    # 开启GUI线程
    t = Thread(target=gui_thread, args=(node,))
    t.daemon = True
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()