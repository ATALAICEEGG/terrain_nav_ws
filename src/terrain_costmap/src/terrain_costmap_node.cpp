#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include <algorithm>

class TerrainCostmapNode : public rclcpp::Node
{
public:
  TerrainCostmapNode()
  : Node("terrain_costmap_node")
  {
    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/terrain_analysis_map",
      rclcpp::QoS(10).reliable(),
      std::bind(&TerrainCostmapNode::mapCallback, this, std::placeholders::_1));

    trav_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "/traversability_map", 10);

    cost_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "/terrain_costmap", 10);

    RCLCPP_INFO(this->get_logger(), "Terrain costmap node started");
  }

private:
  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap in_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, in_map);

    // 检查必需层是否存在
    if (!in_map.exists("slope") || !in_map.exists("roughness") ||
        !in_map.exists("step_height") || !in_map.exists("elevation_variance")) {
      RCLCPP_WARN(this->get_logger(), "Missing required layers, skipping");
      return;
    }

    auto& slope = in_map["slope"];
    auto& roughness = in_map["roughness"];
    auto& step_height = in_map["step_height"];
    auto& variance = in_map["elevation_variance"];

    // 归一化阈值
    const double slope_max = 0.6;
    const double roughness_max = 0.15;
    const double step_max = 0.20;
    const double variance_max = 0.05;

    // 权重
    const double w_slope = 0.35;
    const double w_roughness = 0.25;
    const double w_step = 0.25;
    const double w_variance = 0.15;

    // 添加 traversability 和 terrain_cost 层
    in_map.add("traversability", 0.0);
    in_map.add("terrain_cost", 0.0);
    auto& trav = in_map["traversability"];
    auto& cost = in_map["terrain_cost"];

    double trav_min = std::numeric_limits<double>::max();
    double trav_max = std::numeric_limits<double>::lowest();

    for (grid_map::GridMapIterator it(in_map); !it.isPastEnd(); ++it) {
      const int i = it.getLinearIndex();
      const double s = slope(i);
      const double r = roughness(i);
      const double h = step_height(i);
      const double v = variance(i);

      // 归一化并 clamp 到 [0, 1]
      double s_norm = std::clamp(std::abs(s) / slope_max, 0.0, 1.0);
      double r_norm = std::clamp(std::abs(r) / roughness_max, 0.0, 1.0);
      double h_norm = std::clamp(std::abs(h) / step_max, 0.0, 1.0);
      double v_norm = std::clamp(std::abs(v) / variance_max, 0.0, 1.0);

      // 计算风险与可通行性
      double risk = w_slope * s_norm + w_roughness * r_norm + w_step * h_norm + w_variance * v_norm;
      double t = std::clamp(1.0 - risk, 0.0, 1.0);
      double c = std::clamp((1.0 - t) * 255.0, 0.0, 255.0);

      trav(i) = t;
      cost(i) = c;

      trav_min = std::min(trav_min, t);
      trav_max = std::max(trav_max, t);
    }

    // 发布 /traversability_map
    grid_map::GridMap trav_map;
    trav_map.setFrameId(in_map.getFrameId());
    trav_map.setGeometry(in_map.getLength(), in_map.getResolution(), in_map.getPosition());
    trav_map.add("traversability", trav);
    trav_map.setTimestamp(rclcpp::Time().nanoseconds());
    auto trav_msg = grid_map::GridMapRosConverter::toMessage(trav_map);
    trav_pub_->publish(*trav_msg);

    // 发布 /terrain_costmap
    grid_map::GridMap cost_map;
    cost_map.setFrameId(in_map.getFrameId());
    cost_map.setGeometry(in_map.getLength(), in_map.getResolution(), in_map.getPosition());
    cost_map.add("terrain_cost", cost);
    cost_map.setTimestamp(rclcpp::Time().nanoseconds());
    auto cost_msg = grid_map::GridMapRosConverter::toMessage(cost_map);
    cost_pub_->publish(*cost_msg);

    RCLCPP_INFO(this->get_logger(),
      "Traversability: min=%.3f, max=%.3f | Terrain cost: min=%.0f, max=%.0f",
      trav_min, trav_max,
      std::clamp((1.0 - trav_max) * 255.0, 0.0, 255.0),
      std::clamp((1.0 - trav_min) * 255.0, 0.0, 255.0));
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr trav_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr cost_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainCostmapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
