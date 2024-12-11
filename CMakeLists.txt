cmake_minimum_required(VERSION 3.8)
project(helper_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the library
add_library(helper_functions SHARED src/helper_functions.cpp)

# Include directories
target_include_directories(helper_functions PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
ament_target_dependencies(helper_functions
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  sensor_msgs
)

# Export the library
ament_export_targets(export_helper_functions HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  rclcpp
  rclcpp_components
  composition
  tf2_ros
  tf2
  nav_msgs
  geometry_msgs
  sensor_msgs
  rosidl_default_generators
)

# Add an example executable for testing
add_executable(cli_node src/cli_node.cpp)

# Link the helper_functions library and dependencies to the executable
target_link_libraries(cli_node helper_functions)
ament_target_dependencies(cli_node rclcpp)

# Install the example executable
install(TARGETS cli_node
    DESTINATION lib/${PROJECT_NAME})

# Install rules for the library
install(TARGETS helper_functions
    EXPORT export_helper_functions
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
    DESTINATION include
)

ament_package()
