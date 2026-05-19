#!/usr/bin/env python3
"""Test client for SmacPlanner2D ComputePathToPose action."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy.qos


class SmacTestClient(Node):
    def __init__(self):
        super().__init__('smac_test_client')
        self.get_logger().info('Smac Test Client started')

        self._action_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        self._result_publisher = self.create_publisher(
            Path,
            '/global_path_smac',
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )

        self._goal_sent = False

    def send_goal(self):
        if self._goal_sent:
            return
        self._goal_sent = True

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp.sec = 0
        goal_pose.header.stamp.nanosec = 0
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.planner_id = 'GridBased'

        self.get_logger().info('Sending goal: start=(0,0), goal=(2,2)')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().debug('Received feedback')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        path = result.path
        self.get_logger().info(f'Path received with {len(path.poses)} poses')

        path_msg = Path()
        path_msg.header = path.header
        path_msg.poses = path.poses

        self._result_publisher.publish(path_msg)
        self.get_logger().info('Published to /global_path_smac')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SmacTestClient()

    try:
        node.send_goal()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
