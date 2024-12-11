#ifndef HELPER_FUNCTIONS_HPP
#define HELPER_FUNCTIONS_HPP

#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace helper_package {

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;

// Example helper function
std::string format_message(const std::string &message);

// Function to find index for a given angle and specific points
int find_index_scan_msg(const LaserScan::SharedPtr scan_msg, double angle=0.0);

// Function to normalize angle to [-PI, PI]
double normalize_angle(double angle);

// Functions to convert degrees to radians and vice versa
double rad2deg(double rad);
double deg2rad(double deg);

}  // namespace helper_package

#endif  // HELPER_FUNCTIONS_HPP
