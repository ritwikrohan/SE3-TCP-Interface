#include "hardware_interface/sensor_interface.hpp"
#include "gz_ros2_control/gz_system_interface.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/serialization.hpp>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <rclcpp/serialization.hpp>
#include <fcntl.h>     // for fcntl, F_GETFL, F_SETFL, O_NONBLOCK
#include <unistd.h>    // for close()
#include <errno.h>     // for errno

// SensorInterface becomes redundant here as IgnitionSystemInterface derives from SystemInterface. Still kept it as it was a part os Assignment 7 (doesnt create any issues)

namespace assignment7_sensor_hardware
{
    class SE3HardwareInterface : public hardware_interface::SensorInterface, public ign_ros2_control::IgnitionSystemInterface
    {
        public:

            SE3HardwareInterface();
            ~SE3HardwareInterface();

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
            // I have added below functions because now I am using IgnitionSystemInterface also as base class. They will be empty
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
            bool initSim(rclcpp::Node::SharedPtr &node, std::map<std::string, ignition::gazebo::Entity> &joints, const hardware_interface::HardwareInfo &hardware_info, ignition::gazebo::EntityComponentManager &ecm, int &update_rate) override;

        private:
            std::string ip_address_;
            // Now considering multiple sensor in the interface - Assignment 7 improvement
            std::vector<std::string> sensor_names_;
            std::vector<std::string> sensor_ids_;
            std::vector<std::vector<double>> hw_sensor_states_;
            int sockfd_;
            rclcpp::Serialization<geometry_msgs::msg::PoseStamped> deserialization_;
            bool printed_tool_ = false;
            bool printed_target_ = false;


    };

}