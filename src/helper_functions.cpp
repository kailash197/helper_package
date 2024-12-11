#include "helper_package/helper_functions.hpp"

namespace helper_package {

std::string format_message(const std::string &message) {
    return "[Formatted]: " + message;
}

// Function to find index for a given angle and specific points
int find_index_scan_msg(const LaserScan::SharedPtr scan_msg, double angle) {
    if (!scan_msg) {
        RCLCPP_ERROR(rclcpp::get_logger("find_index_scan_msg"), "Invalid LaserScan message.");
        return -1;
    }

    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
        std::ostringstream error_msg;
        error_msg << "Angle out of bounds, [" << scan_msg->angle_min << " ," << scan_msg->angle_max << "]";
        throw std::invalid_argument(error_msg.str());
    }

    auto angle_to_index = [&](double ang) -> int {
        return static_cast<int>((ang - scan_msg->angle_min) / scan_msg->angle_increment);
    };

    int temp = angle_to_index(angle);
    RCLCPP_DEBUG(rclcpp::get_logger("find_index_scan_msg"), "Angle: %f, Index: %d", angle, temp);
    return temp;
}

// Function to normalize angle to [-PI, PI]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Functions to convert degrees to radians and vice versa
double rad2deg(double rad) { return rad * 180.0 / M_PI; }
double deg2rad(double deg) { return deg * M_PI / 180.0; }

}  // namespace helper_package
