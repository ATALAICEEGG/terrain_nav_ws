# 2.5D 地形导航方案

项目目标：在笔记本上先跑通机器狗 2.5D 地形导航的步骤 1–4 最小闭环，不连接真机。

## 总体流程

输入：2.5D 地形图或模拟 elevation 地形
→ grid_map 多层地图
→ slope / roughness / step / variance
→ traversability / terrain_cost
→ Nav2 SmacPlanner2D
→ 输出 /global_path

## 当前阶段只做

1. 步骤一：2.5D 地形图标准化
2. 步骤二：地形特征计算
3. 步骤三：生成 traversability 与 terrain_cost
4. 步骤四：Global Plan

## 当前阶段不做

- Nav2 Regulated Pure Pursuit Controller
- Local Plan
- local_costmap
- /cmd_vel
- motion_adapter_node
- safety_supervisor_node
- 机器狗真机控制
- elevation_mapping_cupy
- GPU 点云实时建图

## 当前阶段目标

1. 不使用真实 2.5D 数据集，先用模拟 elevation 地图。
2. 不连接机器狗真机。
3. 不使用 elevation_mapping_cupy。
4. 先实现模拟地形图发布、地形特征计算、terrain_cost 生成。
5. 再接入 Nav2 global_costmap、planner_server 和 SmacPlanner2D，只生成 /global_path。

## 需要使用

- ROS2 Humble
- Nav2
- grid_map
- grid_map_costmap_2d
- robot_localization
- tf2

## 当前阶段包与节点

1. terrain_simulation：负责发布模拟 elevation grid_map。
   - terrain_map_publisher_node：发布 /terrain_grid_map。

2. terrain_analysis：负责地形特征计算。
   - terrain_analyzer_node：订阅 /terrain_grid_map，计算 slope、roughness、step、variance，发布 /terrain_grid_map_with_features。

3. terrain_costmap：负责生成 traversability 和 terrain_cost。
   - terrain_costmap_node：订阅 /terrain_grid_map_with_features，发布 /traversability_map 和 /terrain_costmap。

4. terrain_nav_bringup：负责统一启动和 Nav2 全局规划配置。
   - 存放 launch 文件和 Nav2 planner_server / SmacPlanner2D 配置。
   - 当前只输出 /global_path，不输出 /cmd_vel。