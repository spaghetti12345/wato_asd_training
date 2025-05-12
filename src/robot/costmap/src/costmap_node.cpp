#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap") {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10,
  std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap
  costmap_core_.initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      double range = scan->ranges[i];
      if (range < scan->range_max && range > scan->range_min) {
          // Calculate grid coordinates
          int x_grid, y_grid;
          costmap_core_.convertToGrid(range, angle, x_grid, y_grid);
          costmap_core_.markObstacle(x_grid, y_grid);
      }
  }

  // Step 3: Inflate obstacles
  costmap_core_.inflateObstacles();

  // Step 4: Publish costmap
  costmap_core_.publishCostmap(costmap_pub_, scan);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}