from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class OdomImuSimNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_imu_sim_node')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_rate_hz', 10.0)

        self._odom_topic = self.get_parameter('odom_topic').value
        self._imu_topic = self.get_parameter('imu_topic').value
        self._rate = float(self.get_parameter('publish_rate_hz').value)

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._imu_pub = self.create_publisher(Imu, self._imu_topic, 10)

        self._time = 0.0
        self._timer = self.create_timer(1.0 / max(self._rate, 1.0), self._on_timer)

        self.get_logger().info(
            f'odom/imu sim started: odom={self._odom_topic}, imu={self._imu_topic}'
        )

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        half = yaw * 0.5
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q

    def _on_timer(self) -> None:
        self._time += 1.0 / max(self._rate, 1.0)

        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(0.0)
        odom.pose.covariance[0] = 0.05
        self._odom_pub.publish(odom)

        imu = Imu()
        imu.header.frame_id = 'base_link'
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.orientation = self._yaw_to_quaternion(0.0)
        self._imu_pub.publish(imu)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomImuSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
