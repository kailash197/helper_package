#include <rclcpp/rclcpp.hpp>
#include "helper_package/helper_functions.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: ros2 run helper_package cli_node <message>" << std::endl;
        return 1;
    }

    std::string input_message = argv[1];
    std::string formatted_message = helper_package::format_message(input_message);

    std::cout << "Formatted Message: " << formatted_message << std::endl;

    rclcpp::shutdown();
    return 0;
}
