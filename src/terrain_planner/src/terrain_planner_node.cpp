#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <vector>
#include <queue>
#include <functional>
#include <limits>

class TerrainPlannerNode : public rclcpp::Node
{
public:
  TerrainPlannerNode()
  : Node("terrain_planner_node"),
    planned_(false)
  {
    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/terrain_costmap",
      rclcpp::QoS(10).reliable(),
      std::bind(&TerrainPlannerNode::costmapCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/global_path", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TerrainPlannerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Terrain planner node started");
    RCLCPP_INFO(this->get_logger(), "Waiting for /terrain_costmap to plan from (-2,-2) to (2,2)...");
  }

private:
  struct AStarNode
  {
    int ix, iy;
    double g, h;
    int parent_ix, parent_iy;
    bool operator>(const AStarNode& other) const { return (g + h) > (other.g + other.h); }
  };

  void costmapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    if (planned_) return;

    grid_map::GridMap cmap;
    grid_map::GridMapRosConverter::fromMessage(*msg, cmap);

    if (!cmap.exists("terrain_cost")) {
      RCLCPP_WARN(this->get_logger(), "terrain_cost layer not found, skipping");
      return;
    }

    auto& cost_layer = cmap["terrain_cost"];
    const grid_map::Size size = cmap.getSize();
    const double resolution = cmap.getResolution();

    RCLCPP_INFO(this->get_logger(), "Costmap received: %dx%d, resolution=%.3f",
                size(0), size(1), resolution);

    // 固定起终点（世界坐标）
    grid_map::Position start_pos(-2.0, -2.0);
    grid_map::Position goal_pos(2.0, 2.0);

    grid_map::Index start_idx, goal_idx;
    if (!cmap.getIndex(start_pos, start_idx)) {
      RCLCPP_ERROR(this->get_logger(), "Start position (-2,-2) is outside the map!");
      return;
    }
    if (!cmap.getIndex(goal_pos, goal_idx)) {
      RCLCPP_ERROR(this->get_logger(), "Goal position (2,2) is outside the map!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start grid: (%d, %d), Goal grid: (%d, %d)",
                start_idx(0), start_idx(1), goal_idx(0), goal_idx(1));

    // A* 搜索
    auto path_indices = astarSearch(cmap, cost_layer, size, start_idx, goal_idx);

    if (path_indices.empty()) {
      RCLCPP_ERROR(this->get_logger(), "A* planning failed! No path found from (-2,-2) to (2,2)");
      return;
    }

    // 构建 nav_msgs/Path
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (const auto& idx : path_indices) {
      grid_map::Position pos;
      cmap.getPosition(idx, pos);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = this->now();
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    last_path_ = path_msg;
    planned_ = true;

    RCLCPP_INFO(this->get_logger(),
      "A* planning succeeded! Path has %zu points, start=(%.2f,%.2f), goal=(%.2f,%.2f)",
      path_msg.poses.size(),
      path_msg.poses.front().pose.position.x, path_msg.poses.front().pose.position.y,
      path_msg.poses.back().pose.position.x, path_msg.poses.back().pose.position.y);
  }

  std::vector<grid_map::Index> astarSearch(
    const grid_map::GridMap& /*cmap*/,
    const grid_map::Matrix& cost_layer,
    const grid_map::Size& size,
    const grid_map::Index& start,
    const grid_map::Index& goal)
  {
    const int rows = size(0);
    const int cols = size(1);
    const double obstacle_threshold = 200.0;
    const double cost_weight = 0.01;  // terrain_cost 在移动代价中的权重

    // 8 邻域：dx, dy, 移动代价
    const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
    const double move_cost[8] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

    // g_score 初始化
    std::vector<double> g_score(rows * cols, std::numeric_limits<double>::infinity());
    std::vector<bool> closed(rows * cols, false);
    std::vector<int> parent(rows * cols, -1);

    auto toFlat = [cols](int i, int j) { return i * cols + j; };
    auto heuristic = [&](int i, int j) {
      return std::sqrt(std::pow(i - goal(0), 2) + std::pow(j - goal(1), 2));
    };

    int si = start(0), sj = start(1);
    int gi = goal(0), gj = goal(1);

    // 检查起终点是否为障碍
    if (cost_layer(si, sj) > obstacle_threshold) {
      RCLCPP_WARN(this->get_logger(), "Start is on an obstacle (cost=%.0f)", cost_layer(si, sj));
      return {};
    }
    if (cost_layer(gi, gj) > obstacle_threshold) {
      RCLCPP_WARN(this->get_logger(), "Goal is on an obstacle (cost=%.0f)", cost_layer(gi, gj));
      return {};
    }

    g_score[toFlat(si, sj)] = 0.0;
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    open.push({si, sj, 0.0, heuristic(si, sj), -1, -1});

    while (!open.empty()) {
      AStarNode cur = open.top();
      open.pop();

      int ci = cur.ix, cj = cur.iy;
      int flat = toFlat(ci, cj);

      if (closed[flat]) continue;
      closed[flat] = true;

      if (ci == gi && cj == gj) {
        // 回溯路径
        std::vector<grid_map::Index> path;
        int f = flat;
        while (f != -1) {
          path.push_back(grid_map::Index(f / cols, f % cols));
          f = parent[f];
        }
        std::reverse(path.begin(), path.end());
        return path;
      }

      for (int k = 0; k < 8; ++k) {
        int ni = ci + dx[k];
        int nj = cj + dy[k];
        if (ni < 0 || ni >= rows || nj < 0 || nj >= cols) continue;

        int nflat = toFlat(ni, nj);
        if (closed[nflat]) continue;

        double tc = cost_layer(ni, nj);
        if (tc > obstacle_threshold) continue;

        double new_g = g_score[flat] + move_cost[k] + cost_weight * tc;
        if (new_g < g_score[nflat]) {
          g_score[nflat] = new_g;
          parent[nflat] = flat;
          open.push({ni, nj, new_g, heuristic(ni, nj), ci, cj});
        }
      }
    }

    return {};  // 无路径
  }

  void timerCallback()
  {
    if (!planned_ || last_path_.poses.empty()) return;
    last_path_.header.stamp = this->now();
    path_pub_->publish(last_path_);
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool planned_;
  nav_msgs::msg::Path last_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
