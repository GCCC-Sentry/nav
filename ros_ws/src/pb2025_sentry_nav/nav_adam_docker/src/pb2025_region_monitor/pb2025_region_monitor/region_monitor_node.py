import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RegionMonitorNode(Node):
    def __init__(self):
        super().__init__('region_monitor_node')

        # ================= 配置区域 (多边形顶点，按顺序排列) =================
        self.regions = [
            {
                'name': 'patrol_zone_1',
                'vertices': [
                    (-3.21, -6.21),
                    (-1.59, -5.96),
                    (-1.07, -4.14),
                    (-3.18, -4.36),
                ]
            },
            # 您可以添加更多多边形区域...
        ]
        # ===================================================================

        # 发布可视化 Marker 的话题
        self.marker_pub = self.create_publisher(MarkerArray, '/region_markers', 10)

        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 状态记录
        self.in_region_prev = False

        # 定时器 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('区域监控节点已启动 (仅可视化)')

    @staticmethod
    def point_in_polygon(px, py, vertices):
        """射线法判断点是否在多边形内"""
        n = len(vertices)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = vertices[i]
            xj, yj = vertices[j]
            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def timer_callback(self):
        try:
            # 1. 获取机器人坐标
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time())

            current_x = t.transform.translation.x
            current_y = t.transform.translation.y

            # 2. 检查区域并发布可视化
            in_any_region = False
            marker_array = MarkerArray()
            timestamp = self.get_clock().now().to_msg()

            for i, region in enumerate(self.regions):
                verts = region['vertices']
                is_inside_this = self.point_in_polygon(current_x, current_y, verts)

                if is_inside_this:
                    in_any_region = True

                # === 构建可视化 Marker ===
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = timestamp
                marker.ns = "monitor_regions"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.05  # 线条宽度

                # 颜色逻辑：进入变红，平时绿
                if is_inside_this:
                    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
                else:
                    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
                marker.color.a = 1.0

                # 按顶点顺序绘制多边形 (首尾相连)
                for vx, vy in verts:
                    marker.points.append(Point(x=vx, y=vy, z=0.0))
                marker.points.append(Point(x=verts[0][0], y=verts[0][1], z=0.0))

                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 200000000  # 0.2s

                marker_array.markers.append(marker)

            # 发布可视化 Marker
            self.marker_pub.publish(marker_array)

            # 3. 日志提示（仅状态变化时打印）
            if in_any_region and not self.in_region_prev:
                self.get_logger().warn('>>> 进入监控区域')
            elif not in_any_region and self.in_region_prev:
                self.get_logger().info('<<< 离开监控区域')
            self.in_region_prev = in_any_region

        except TransformException:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RegionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
