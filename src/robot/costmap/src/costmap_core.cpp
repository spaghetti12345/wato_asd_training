#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>

CostmapCore::CostmapCore() {}

void CostmapCore::initializeCostmap() {
  costmap_ = std::vector<std::vector<int>>(height_, std::vector<int>(width_, 0));
}

void CostmapCore::convertToGrid(double range, double angle, int &x, int &y) {
  double grid_x = range * std::cos(angle);
  double grid_y = range * std::sin(angle);

  x = static_cast<int>((grid_x - origin_x_) / resolution_);
  y = static_cast<int>((grid_y - origin_y_) / resolution_);
}

void CostmapCore::markObstacle(int x, int y) {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    costmap_[y][x] = max_cost_;
  }
}

void CostmapCore::inflateObstacles() {
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
  std::vector<std::vector<int>> new_map = costmap_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (costmap_[y][x] == max_cost_) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
              double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
              if (distance <= inflation_radius_) {
                int inflated_cost = static_cast<int>(max_cost_ * (1.0 - (distance / inflation_radius_)));
                new_map[ny][nx] = std::max(new_map[ny][nx], inflated_cost);
              }
            }
          }
        }
      }
    }
  }
  costmap_ = new_map;
}

void CostmapCore::publishCostmap(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_, sensor_msgs::msg::LaserScan::SharedPtr scan) {
  nav_msgs::msg::OccupancyGrid grid_msg;

  grid_msg.header.stamp = rclcpp::Clock().now();
  grid_msg.header = scan->header; 

  grid_msg.info.resolution = resolution_;
  grid_msg.info.width = width_;
  grid_msg.info.height = height_;
  grid_msg.info.origin.position.x = origin_x_;
  grid_msg.info.origin.position.y = origin_y_;
  grid_msg.info.origin.orientation.w = 1.0;  // Identity quaternion

  grid_msg.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      grid_msg.data[y * width_ + x] = costmap_[y][x];
    }
  }

  costmap_pub_->publish(grid_msg);
}
