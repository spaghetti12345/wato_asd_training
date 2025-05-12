#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>

class MapMemoryCore {
public:
    MapMemoryCore();
    void initializeMap(int width, int height, double resolution);
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid &costmap, double robot_x, double robot_y);
    const nav_msgs::msg::OccupancyGrid &getMap() const;

private:
    nav_msgs::msg::OccupancyGrid global_map_;
    bool initialized_ = false;
};

#endif
