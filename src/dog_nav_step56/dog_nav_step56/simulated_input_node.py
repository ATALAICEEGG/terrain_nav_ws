from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class SimulatedInputNode(Node):
    def __init__(self) -> None:
        super().__init__('simulated_input_node')

        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('terrain_cost_topic', '/terrain_cost')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('path_length', 12)
        self.declare_parameter('path_spacing', 0.5)
        self.declare_parameter('terrain_cost', 20.0)
        self.declare_parameter('simulate_tilt', False)

        self._path_topic = self.get_parameter('path_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._imu_topic = self.get_parameter('imu_topic').value
        self._terrain_cost_topic = self.get_parameter('terrain_cost_topic').value
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._path_length = int(self.get_parameter('path_length').value)
        self._path_spacing = float(self.get_parameter('path_spacing').value)
        self._terrain_cost = float(self.get_parameter('terrain_cost').value)
        self._simulate_tilt = self._as_bool(self.get_parameter('simulate_tilt').value)

        self._path_pub = self.create_publisher(Path, self._path_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._imu_pub = self.create_publisher(Imu, self._imu_topic, 10)
        self._terrain_pub = self.create_publisher(Float32, self._terrain_cost_topic, 10)

        self._time = 0.0
        self._timer = self.create_timer(1.0 / max(self._publish_rate_hz, 1.0), self._on_timer)
        self.get_logger().info(
            f'simulated input started: path={self._path_topic}, odom={self._odom_topic}, imu={self._imu_topic}, terrain={self._terrain_cost_topic}'
        )

    def _as_bool(self, value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('true', '1', 'yes', 'on')
        return bool(value)

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        half = yaw * 0.5
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(half)
        quat.w = math.cos(half)
        return quat

    def _build_path(self) -> Path:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for index in range(self._path_length):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = index * self._path_spacing
            pose.pose.position.y = 0.25 * math.sin(index * 0.4)
            pose.pose.position.z = 0.0
            pose.pose.orientation = self._yaw_to_quaternion(0.0)
            path.poses.append(pose)
        return path

    def _build_odom(self) -> Odometry:
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = 0.25 * self._time
        odom.pose.pose.position.y = 0.15 * math.sin(self._time * 0.6)
        odom.pose.pose.position.z = 0.0
        yaw = 0.12 * math.sin(self._time * 0.4)
        odom.pose.pose.orientation = self._yaw_to_quaternion(yaw)
        odom.pose.covariance[0] = 0.05 + 0.02 * abs(math.sin(self._time * 0.5))
        return odom

    def _build_imu(self) -> Imu:
        imu = Imu()
        imu.header.frame_id = 'base_link'
        imu.header.stamp = self.get_clock().now().to_msg()
        tilt = 0.03 * math.sin(self._time * 0.7)
        if self._simulate_tilt and 8.0 < self._time < 10.0:
            tilt = 0.38
        half = tilt * 0.5
        imu.orientation.x = math.sin(half)
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = math.cos(half)
        return imu

    def _on_timer(self) -> None:
        self._time += 1.0 / max(self._publish_rate_hz, 1.0)

        self._path_pub.publish(self._build_path())
        self._odom_pub.publish(self._build_odom())
        self._imu_pub.publish(self._build_imu())

        terrain = Float32()
        terrain.data = float(self._terrain_cost if self._time < 7.0 else min(240.0, self._terrain_cost + 100.0))
        self._terrain_pub.publish(terrain)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulatedInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
