#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{
class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger); // Constructor

    // Initialize the control core (with parameters)
    void initControlCore(
      double lookahead_distance,
      double max_steering_angle, 
      double steering_gain,
      double linear_velocity
    );

    bool isPathEmpty();
    void updatePath(nav_msgs::msg::Path path);
    unsigned int findLookaheadPoint(double robot_x, double robot_y, double robot_theta);
    
    // Calculate the control command based on the robot's position and orientation
    geometry_msgs::msg::Twist calculateControlCommand(double robot_x, double robot_y, double robot_theta);
  
  private:
    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;

    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double linear_velocity_;
};

} 
#endif