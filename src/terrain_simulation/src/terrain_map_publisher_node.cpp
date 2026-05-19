#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>

class TerrainMapPublisherNode : public rclcpp::Node
{
public:
  TerrainMapPublisherNode()
  : Node("terrain_map_publisher_node")
  {
    map_.setFrameId("map");
    map_.setGeometry(grid_map::Length(5.0, 5.0), 0.05, grid_map::Position(0.0, 0.0));
    map_.add("elevation");
    map_.add("elevation_variance", 0.01f);
    generateGaussianTerrain();
    
    publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "/terrain_grid_map", 10);
    
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TerrainMapPublisherNode::publishMap, this));
    
    RCLCPP_INFO(this->get_logger(), "Terrain map publisher node started");
  }

private:
  void generateGaussianTerrain()
  {
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map_.getPosition(*it, position);
      double x = position.x();
      double y = position.y();
      double sigma = 1.5;
      double elevation = 0.5 * exp(-(x*x + y*y) / (2 * sigma * sigma));
      map_.at("elevation", *it) = elevation;
    }
  }
  
  void publishMap()
  {
    map_.setTimestamp(rclcpp::Time().nanoseconds());
    auto message = grid_map::GridMapRosConverter::toMessage(map_);
    publisher_->publish(*message);
  }
  
  grid_map::GridMap map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainMapPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
