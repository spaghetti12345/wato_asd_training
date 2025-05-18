#ifndef MAP_MEMORY_CORE_HPP
#define MAP_MEMORY_CORE_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>


class MapMemoryCore {
public:
    MapMemoryCore();

    void initializeMap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin);

    void integrateLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& local_map,
                            double robot_x, double robot_y, double robot_theta);

    nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalMap() const;

private:
    bool mapCoordinatesToGlobal(double world_x, double world_y, int& global_x, int& global_y) const;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    std::vector<bool> updated_cells_;

    double last_robot_x_;
    double last_robot_y_;

    double move_threshold_;
    
    double computeDistance(double x1, double y1, double x2, double y2) const;
};


#endif