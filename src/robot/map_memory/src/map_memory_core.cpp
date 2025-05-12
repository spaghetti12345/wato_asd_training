#include "map_memory_core.hpp"

MapMemoryCore::MapMemoryCore() {}

void MapMemoryCore::initializeMap(int width, int height, double resolution) {
    global_map_.info.width = width;
    global_map_.info.height = height;
    global_map_.info.resolution = resolution;
    global_map_.info.origin.position.x = -width * resolution / 2.0;
    global_map_.info.origin.position.y = -height * resolution / 2.0;
    global_map_.data.resize(width * height, -1);  // Unknown
    initialized_ = true;
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid &costmap, double robot_x, double robot_y) {
    if (!initialized_) return;

    int map_width = global_map_.info.width;
    int map_height = global_map_.info.height;
    double res = global_map_.info.resolution;

    int origin_x = static_cast<int>((robot_x - costmap.info.width * res / 2.0 - global_map_.info.origin.position.x) / res);
    int origin_y = static_cast<int>((robot_y - costmap.info.height * res / 2.0 - global_map_.info.origin.position.y) / res);

    for (unsigned int y = 0; y < costmap.info.height; ++y) {
        for (unsigned int x = 0; x < costmap.info.width; ++x) {
            int local_index = y * costmap.info.width + x;
            int gx = origin_x + x;
            int gy = origin_y + y;
            if (gx >= 0 && gx < map_width && gy >= 0 && gy < map_height) {
                int global_index = gy * map_width + gx;
                int cost = costmap.data[local_index];
                if (cost != -1) {
                    global_map_.data[global_index] = cost;
                }
            }
        }
    }
}

const nav_msgs::msg::OccupancyGrid &MapMemoryCore::getMap() const {
    return global_map_;
}

