#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include <limits>
#include <vector>

class TerrainAnalyzerNode : public rclcpp::Node
{
public:
  TerrainAnalyzerNode()
  : Node("terrain_analyzer_node")
  {
    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/terrain_grid_map",
      rclcpp::QoS(10).reliable(),
      std::bind(&TerrainAnalyzerNode::mapCallback, this, std::placeholders::_1));
    
    pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "/terrain_analysis_map", 10);
    
    RCLCPP_INFO(this->get_logger(), "Terrain analyzer node started");
  }

private:
  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "=== Terrain Map Info ===");
    RCLCPP_INFO(this->get_logger(), "Length X: %.2f m", msg->info.length_x);
    RCLCPP_INFO(this->get_logger(), "Length Y: %.2f m", msg->info.length_y);
    RCLCPP_INFO(this->get_logger(), "Resolution: %.3f m", msg->info.resolution);
    std::string layers_str;
    for (const auto& layer : msg->layers) {
      layers_str += layer + " ";
    }
    RCLCPP_INFO(this->get_logger(), "Layers: [%s]", layers_str.c_str());
    
    // 转换消息为 GridMap 对象
    grid_map::GridMap out_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, out_map);
    
    // 获取 elevation 层和分辨率
    const double resolution = out_map.getResolution();
    auto& elevation = out_map["elevation"];
    const grid_map::Size size = out_map.getSize();
    
    // 添加 slope 层并用有限差分法计算坡度
    out_map.add("slope", 0.0);
    auto& slope = out_map["slope"];
    
    double slope_min = std::numeric_limits<double>::max();
    double slope_max = std::numeric_limits<double>::lowest();
    
    for (int i = 1; i < size(0) - 1; ++i) {
      for (int j = 1; j < size(1) - 1; ++j) {
        double dx = (elevation(i + 1, j) - elevation(i - 1, j)) / (2.0 * resolution);
        double dy = (elevation(i, j + 1) - elevation(i, j - 1)) / (2.0 * resolution);
        double slope_val = std::atan2(std::sqrt(dx * dx + dy * dy), 1.0);
        slope(i, j) = slope_val;
        slope_min = std::min(slope_min, slope_val);
        slope_max = std::max(slope_max, slope_val);
      }
    }
    
    // 边界格子保持为 0（已初始化）
    
    // 计算 roughness（3×3 邻域标准差）
    out_map.add("roughness", 0.0);
    auto& roughness = out_map["roughness"];
    
    double roughness_min = std::numeric_limits<double>::max();
    double roughness_max = std::numeric_limits<double>::lowest();
    
    for (int i = 0; i < size(0); ++i) {
      for (int j = 0; j < size(1); ++j) {
        // 收集 3×3 邻域数据
        std::vector<double> neighbors;
        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            int ni = i + di;
            int nj = j + dj;
            if (ni >= 0 && ni < size(0) && nj >= 0 && nj < size(1)) {
              neighbors.push_back(elevation(ni, nj));
            }
          }
        }
        
        if (neighbors.size() >= 2) {
          // 计算标准差
          double mean = 0.0;
          for (double v : neighbors) mean += v;
          mean /= neighbors.size();
          
          double var = 0.0;
          for (double v : neighbors) var += (v - mean) * (v - mean);
          double std_dev = std::sqrt(var / neighbors.size());
          
          roughness(i, j) = std_dev;
          roughness_min = std::min(roughness_min, std_dev);
          roughness_max = std::max(roughness_max, std_dev);
        }
      }
    }
    
    // 计算 step_height（4-邻域最大高度差）
    out_map.add("step_height", 0.0);
    auto& step_height = out_map["step_height"];
    
    double step_min = std::numeric_limits<double>::max();
    double step_max = std::numeric_limits<double>::lowest();
    
    const int di[4] = {-1, 1, 0, 0};
    const int dj[4] = {0, 0, -1, 1};
    
    for (int i = 0; i < size(0); ++i) {
      for (int j = 0; j < size(1); ++j) {
        double max_diff = 0.0;
        for (int k = 0; k < 4; ++k) {
          int ni = i + di[k];
          int nj = j + dj[k];
          if (ni >= 0 && ni < size(0) && nj >= 0 && nj < size(1)) {
            double diff = std::abs(elevation(i, j) - elevation(ni, nj));
            max_diff = std::max(max_diff, diff);
          }
        }
        step_height(i, j) = max_diff;
        step_min = std::min(step_min, max_diff);
        step_max = std::max(step_max, max_diff);
      }
    }
    
    // 发布结果
    out_map.setTimestamp(rclcpp::Time().nanoseconds());
    auto out_msg = grid_map::GridMapRosConverter::toMessage(out_map);
    pub_->publish(*out_msg);
    
    RCLCPP_INFO(this->get_logger(), "Slope layer added: min=%.4f rad, max=%.4f rad",
                slope_min, slope_max);
    RCLCPP_INFO(this->get_logger(), "Roughness layer added: min=%.6f m, max=%.6f m",
                roughness_min, roughness_max);
    RCLCPP_INFO(this->get_logger(), "Step height layer added: min=%.6f m, max=%.6f m",
                step_min, step_max);
  }
  
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainAnalyzerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
