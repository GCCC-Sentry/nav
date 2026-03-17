#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateThroughPoses
from std_msgs.msg import String


class NavThroughPosesClient(Node):
    def __init__(self):
        super().__init__('nav_through_poses_client')

        # Action client for NavigateThroughPoses
        self._action_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )

        # Subscribe to clicked points from RViz2
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        # Store collected poses
        self.poses = []

        # Create a service or topic to trigger navigation
        self.trigger_sub = self.create_subscription(
            String,
            '/nav_trigger',
            self.trigger_navigation,
            10
        )

        self.get_logger().info('Nav Through Poses Client initialized')
        self.get_logger().info('Click points in RViz2, then publish to /nav_trigger to start navigation')

    def clicked_point_callback(self, msg):
        """Callback for clicked points from RViz2"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0  # Default orientation

        self.poses.append(pose)
        self.get_logger().info(f'Added pose {len(self.poses)}: ({msg.point.x:.2f}, {msg.point.y:.2f})')

    def trigger_navigation(self, msg):
        """Trigger navigation through collected poses"""
        if len(self.poses) == 0:
            self.get_logger().warn('No poses collected yet!')
            return

        self.get_logger().info(f'Starting navigation through {len(self.poses)} poses')
        self.send_goal()

    def send_goal(self):
        """Send goal to NavigateThroughPoses action server"""
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.poses

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current pose index: {feedback.current_pose}')

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info('Navigation completed!')
        # Clear poses after successful navigation
        self.poses.clear()


def main(args=None):
    rclpy.init(args=args)
    node = NavThroughPosesClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
