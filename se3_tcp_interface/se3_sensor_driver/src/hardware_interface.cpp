#include "se3_sensor_driver/hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Reference https://github.com/ros-controls/ros2_control_demos/blob/humble/example_5/hardware/external_rrbot_force_torque_sensor.cpp
namespace assignment7_sensor_hardware
{

    SE3HardwareInterface::SE3HardwareInterface(){
        std::cout << "SE3 Hardware Interface constructed!!" << std::endl;
    }

    SE3HardwareInterface::~SE3HardwareInterface(){
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
        std::cout << "SE3 Hardware Interface Exit!!" << std::endl;
    }

    hardware_interface::CallbackReturn SE3HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {

        if( hardware_interface::SensorInterface::on_init( info ) != hardware_interface::CallbackReturn::SUCCESS ){
            return hardware_interface::CallbackReturn::ERROR;
        }

        if( hardware_interface::SystemInterface::on_init( info ) != hardware_interface::CallbackReturn::SUCCESS ){
            return hardware_interface::CallbackReturn::ERROR;
        }

        sensor_names_.resize(hardware_interface::SystemInterface::info_.sensors.size());
        sensor_ids_.resize(hardware_interface::SystemInterface::info_.sensors.size());
        hw_sensor_states_.resize(hardware_interface::SystemInterface::info_.sensors.size());
        ip_address_ = (hardware_interface::SystemInterface::info_.hardware_parameters["ip_address"]);

        struct protoent *protoent = getprotobyname("tcp");
        sockfd_ = socket(AF_INET, SOCK_STREAM, protoent->p_proto);
        struct hostent *hostent = gethostbyname(ip_address_.c_str());
        if (!hostent) {
            std::cerr << "gethostbyname() failed for IP: " << ip_address_ << std::endl;
            return hardware_interface::CallbackReturn::ERROR;
        }
        struct sockaddr_in sockaddr;
        sockaddr.sin_addr.s_addr = inet_addr(inet_ntoa(*(struct in_addr*)*(hostent->h_addr_list)));
        sockaddr.sin_family = AF_INET;
        sockaddr.sin_port = htons(12345);
        
        if (connect(sockfd_, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) == -1) {
            std::cout << "Failed to connect to TCP server at " << ip_address_ << std::endl;
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        std::cout << "Connected to TCP server at " << ip_address_ << std::endl;
        
        
        for (size_t i = 0; i<hardware_interface::SystemInterface::info_.sensors.size(); ++i)
        {   
            sensor_names_[i] = hardware_interface::SystemInterface::info_.sensors[i].name;
            sensor_ids_[i] = (hardware_interface::SystemInterface::info_.sensors[i].parameters["sensor_id"]); // param from sensor or hardware interface any of them can be used
            // sensor_id_ = (hardware_interface::SystemInterface::info_.hardware_parameters["sensor_id"]);
            std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
            std::cout << "SE3 Sensor Initialized!!" << std::endl;
            std::cout << "  Sensor Name: " << sensor_names_[i] << std::endl;
            std::cout << "  Sensor ID: " << sensor_ids_[i] << std::endl;
            std::cout << "  IP Address: " << ip_address_ << std::endl;
            std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
            hw_sensor_states_[i].resize(hardware_interface::SystemInterface::info_.sensors[i].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SE3HardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < hardware_interface::SystemInterface::info_.sensors.size(); ++i) {
            for (size_t j = 0; j < hardware_interface::SystemInterface::info_.sensors[i].state_interfaces.size(); j++)
            {   
                // Improvement from assignment 7. Now considers 0 or multiple sensors
                state_interfaces.emplace_back(hardware_interface::StateInterface(hardware_interface::SystemInterface::info_.sensors[i].name, hardware_interface::SystemInterface::info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_[i][j]));
            }
        }
        return state_interfaces;
    }

    hardware_interface::return_type SE3HardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &)
    {
        rclcpp::SerializedMessage msg(512);
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sockfd_, &readfds);
        struct timeval timeout = {0, 0}; 

        int ready = select(sockfd_ + 1, &readfds, NULL, NULL, &timeout);
        if (ready <= 0) {
            return hardware_interface::return_type::OK;
        }

        ssize_t n = ::read(sockfd_, msg.get_rcl_serialized_message().buffer, 512);
        if (n <= 0) {
            return hardware_interface::return_type::OK;
        }
        msg.get_rcl_serialized_message().buffer_length = static_cast<size_t>(n);

        geometry_msgs::msg::PoseStamped pose;
        try {
            deserialization_.deserialize_message(&msg, &pose);
        } catch (const std::exception &e) {
            std::cerr << "Deserialization failed: " << e.what() << "\n";
            return hardware_interface::return_type::OK;
        }

        for (size_t i = 0; i < sensor_ids_.size(); ++i)
        {
            if ((pose.header.frame_id == "tool_sensor" && sensor_ids_[i] == "tool_sensor_id") ||
                (pose.header.frame_id == "target_sensor" && sensor_ids_[i] == "target_sensor_id"))
            {
                hw_sensor_states_[i][0] = pose.pose.position.x;
                hw_sensor_states_[i][1] = pose.pose.position.y;
                hw_sensor_states_[i][2] = pose.pose.position.z;
                hw_sensor_states_[i][3] = pose.pose.orientation.x;
                hw_sensor_states_[i][4] = pose.pose.orientation.y;
                hw_sensor_states_[i][5] = pose.pose.orientation.z;
                hw_sensor_states_[i][6] = pose.pose.orientation.w;

                if ((sensor_ids_[i] == "tool_sensor_id" && !printed_tool_) ||
                    (sensor_ids_[i] == "target_sensor_id" && !printed_target_))
                {
                    std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
                    std::cout << "Reading Poses for Sensor at time: " << time.seconds() << " sec" << std::endl;
                    std::cout << "  Sensor Name: " << sensor_names_[i] << std::endl;
                    std::cout << "  Sensor ID: " << sensor_ids_[i] << std::endl;
                    std::cout << "  IP Address: " << ip_address_ << std::endl;
                    for (size_t j = 0; j < hw_sensor_states_[i].size(); ++j) {
                        std::cout << std::fixed << std::setprecision(2)
                                << "    " << hw_sensor_states_[i][j]
                                << " for state interface '" << hardware_interface::SystemInterface::info_.sensors[i].state_interfaces[j].name << "'"
                                << std::endl;
                    }
                    std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;

                    if (sensor_ids_[i] == "tool_sensor_id") {
                        printed_tool_ = true;
                    } else {
                        printed_target_ = true;
                    }
                }
            }
        }

        return hardware_interface::return_type::OK;
    }


     bool SE3HardwareInterface::initSim(rclcpp::Node::SharedPtr & ,std::map<std::string, ignition::gazebo::Entity> & ,const hardware_interface::HardwareInfo & ,ignition::gazebo::EntityComponentManager & ,int & )
    {
    return true; // EMPTY because its basically a sensor interface but used IgnitionSystemInterface which needs this function, so had to include it.
    }

    std::vector<hardware_interface::CommandInterface> SE3HardwareInterface::export_command_interfaces()
    {
        return {}; // EMPTY because its basically a sensor interface but used IgnitionSystemInterface (which derives from system interface), so had to include it.
    }

    hardware_interface::return_type SE3HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        return hardware_interface::return_type::OK;   // EMPTY because its basically a sensor interface but used IgnitionSystemInterface (which derives from system interface), so had to include it.
    }
} 

#include "pluginlib/class_list_macros.hpp"

// SensorInterface becomes redundant here but still kept it as it was a part of assignment 7
PLUGINLIB_EXPORT_CLASS(assignment7_sensor_hardware::SE3HardwareInterface, hardware_interface::SensorInterface)
PLUGINLIB_EXPORT_CLASS(assignment7_sensor_hardware::SE3HardwareInterface, ign_ros2_control::IgnitionSystemInterface)