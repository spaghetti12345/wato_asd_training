#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

class CostmapCore {
  public:
    CostmapCore(); //constructor
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x, int &y);
    void markObstacle(int x, int y);
    void inflateObstacles();
    void publishCostmap(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_, sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    std::vector<std::vector<int>> costmap_; //2D array (costmap)
    int width_ = 300;
    int height_ = 300;
    double resolution_ = 0.25;
    double origin_x_ = -25.0;
    double origin_y_ = -25.0;
    double inflation_radius_ = 1.5;
    int max_cost_ = 100;

};


#endif  