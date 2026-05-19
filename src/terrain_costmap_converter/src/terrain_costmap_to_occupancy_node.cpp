#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <memory>

using namespace std::chrono_literals;

class TerrainCostmapConverterNode : public rclcpp::Node
{
public:
  TerrainCostmapConverterNode()
  : Node("terrain_costmap_to_occupancy_node")
  {
    RCLCPP_INFO(this->get_logger(), "Terrain Costmap Converter Node started");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    auto sub_qos = rclcpp::QoS(10).reliable().durability_volatile();

    subscription_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/terrain_costmap",
      sub_qos,
      std::bind(&TerrainCostmapConverterNode::gridMapCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
  }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received terrain_costmap");

    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(*msg, gridMap);

    if (!gridMap.exists("terrain_cost")) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "terrain_cost layer not found in GridMap");
      return;
    }

    const auto& terrainCost = gridMap["terrain_cost"];
    grid_map::Length length = gridMap.getLength();
    grid_map::Position position = gridMap.getPosition();
    double resolution = gridMap.getResolution();
    grid_map::Size size = gridMap.getSize();

    nav_msgs::msg::OccupancyGrid occupancyGrid;
    std::string frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    occupancyGrid.header.frame_id = frame_id;
    occupancyGrid.header.stamp = msg->header.stamp;

    occupancyGrid.info.width = size(0);
    occupancyGrid.info.height = size(1);
    occupancyGrid.info.resolution = resolution;

    occupancyGrid.info.origin.position.x = position.x() - length.x() / 2.0;
    occupancyGrid.info.origin.position.y = position.y() - length.y() / 2.0;
    occupancyGrid.info.origin.position.z = 0.0;
    occupancyGrid.info.origin.orientation.w = 1.0;
    occupancyGrid.info.origin.orientation.x = 0.0;
    occupancyGrid.info.origin.orientation.y = 0.0;
    occupancyGrid.info.origin.orientation.z = 0.0;

    occupancyGrid.data.resize(size(0) * size(1));

    for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      const auto& value = terrainCost(index(0), index(1));

      size_t linearIndex = index(1) * size(0) + index(0);

      if (std::isnan(value)) {
        occupancyGrid.data[linearIndex] = -1;
      } else {
        double normalized = std::clamp(value / 255.0 * 100.0, 0.0, 100.0);
        occupancyGrid.data[linearIndex] = static_cast<int8_t>(std::round(normalized));
      }
    }

    RCLCPP_DEBUG(this->get_logger(),
      "Published OccupancyGrid: %ux%u, resolution=%.3f",
      occupancyGrid.info.width, occupancyGrid.info.height, occupancyGrid.info.resolution);

    publisher_->publish(occupancyGrid);
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainCostmapConverterNode>());
  rclcpp::shutdown();
  return 0;
}
