#include "se3_sensor_driver/sensor_tcp_server.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto sensor_node = std::make_shared<assignment8::sensor_tcp_server>("sensor_tcp_server_node");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sensor_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
