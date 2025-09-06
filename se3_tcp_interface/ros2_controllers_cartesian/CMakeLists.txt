cmake_minimum_required(VERSION 3.8)
project(ros2_controllers_cartesian)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(tf2_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories( include )

add_library(cartesian_motion_controller SHARED src/cartesian_controller.cpp)
target_link_libraries(cartesian_motion_controller tf2_kdl::tf2_kdl)
ament_target_dependencies(cartesian_motion_controller rclcpp Eigen3 pluginlib std_msgs geometry_msgs kdl_parser tf2_kdl tf2_ros tf2_geometry_msgs controller_interface)

pluginlib_export_plugin_description_file(controller_interface cartesian_motion_controller_plugin.xml)

install( TARGETS cartesian_motion_controller LIBRARY DESTINATION lib )


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

ament_package()
