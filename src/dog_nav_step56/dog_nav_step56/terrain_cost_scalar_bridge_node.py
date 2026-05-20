from __future__ import annotations

import math

import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from std_msgs.msg import Float32


class TerrainCostScalarBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('terrain_cost_scalar_bridge_node')

        self.declare_parameter('input_topic', '/terrain_costmap')
        self.declare_parameter('output_topic', '/terrain_cost')

        self._input_topic = self.get_parameter('input_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._first_published = False

        self._sub = self.create_subscription(
            GridMap, self._input_topic, self._on_gridmap, 10
        )
        self._pub = self.create_publisher(Float32, self._output_topic, 10)

        self.get_logger().info(
            f'terrain cost scalar bridge started: {self._input_topic} -> {self._output_topic}'
        )

    def _on_gridmap(self, msg: GridMap) -> None:
        try:
            if 'terrain_cost' not in msg.layers:
                self.get_logger().warn_throttle(
                    5.0,
                    f'terrain_cost layer not found. Available layers: {msg.layers}'
                )
                return

            layer_index = msg.layers.index('terrain_cost')
            raw_data = msg.data[layer_index].data

            valid_values = [
                v for v in raw_data
                if not (math.isnan(v) or math.isinf(v))
            ]

            if not valid_values:
                self.get_logger().warn_throttle(5.0, 'terrain_cost layer has no valid data')
                return

            mean_cost = sum(valid_values) / len(valid_values)
            float_msg = Float32()
            float_msg.data = float(mean_cost)
            self._pub.publish(float_msg)

            if not self._first_published:
                self.get_logger().info(f'published terrain_cost scalar: {mean_cost:.2f}')
                self._first_published = True

        except Exception as e:
            self.get_logger().error(f'failed to process gridmap: {e}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TerrainCostScalarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
