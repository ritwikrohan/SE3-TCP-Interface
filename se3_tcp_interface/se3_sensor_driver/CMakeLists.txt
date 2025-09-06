cmake_minimum_required(VERSION 3.8)
project(se3_sensor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur_description REQUIRED)
find_package(ign_ros2_control REQUIRED)
find_package(ignition-cmake2 REQUIRED)

ign_find_package( ignition-plugin1 REQUIRED)
ign_find_package( ignition-gazebo6 REQUIRED )
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})


include_directories( include  ${ign_ros2_control_INCLUDE_DIRS} )
add_library( se3_driver_plugin SHARED src/hardware_interface.cpp )
ament_target_dependencies( se3_driver_plugin controller_manager controller_manager_msgs hardware_interface pluginlib ign_ros2_control rclcpp )
target_link_libraries(se3_driver_plugin
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::register
)


add_library(sensor_tcp_server_lib src/sensor_tcp_server.cpp)
ament_target_dependencies(sensor_tcp_server_lib rclcpp tf2_ros geometry_msgs)

add_executable(tcp_server src/tcp_server_node.cpp)
target_link_libraries(tcp_server sensor_tcp_server_lib)
ament_target_dependencies(tcp_server rclcpp)


install(DIRECTORY include/ DESTINATION include)
install(TARGETS tcp_server DESTINATION lib/${PROJECT_NAME})
install( TARGETS se3_driver_plugin sensor_tcp_server_lib DESTINATION lib )
pluginlib_export_plugin_description_file(ign_ros2_control se3_interface_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface se3_interface_plugin.xml)

install( DIRECTORY urdf launch rviz config DESTINATION share/${PROJECT_NAME} )

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
