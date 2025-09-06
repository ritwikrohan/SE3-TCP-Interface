#include "se3_sensor_driver/sensor_tcp_server.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace assignment8{
    sensor_tcp_server::sensor_tcp_server(const std::string& name): Node(name){

        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = this->create_wall_timer(10ms,std::bind(&sensor_tcp_server::tf_lookup_timer, this), timer_callback_group_);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // TCP/IP stuff - used from rsp github
        struct protoent *protoent = getprotobyname("tcp");
        sockfd = socket(AF_INET, SOCK_STREAM, protoent->p_proto);
        if (sockfd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open socket.");
            return;
        }
        int enable = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set socket options.");
            return;
        }

        struct sockaddr_in sockaddr;
        sockaddr.sin_family = AF_INET;
        sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        sockaddr.sin_port = htons(12345);

        if (bind(sockfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket.");
            return;
        }

        if (listen(sockfd, 5) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to listen.");
            return;
        }
    }

    sensor_tcp_server::~sensor_tcp_server() {
        if (client_sockfd >= 0) close(client_sockfd);
            if (sockfd >= 0) close(sockfd);
    }


    void sensor_tcp_server::tf_lookup_timer(){

        // waiting for client (controllers)
        if (!client_connected) {
            struct sockaddr_in client_address;
            socklen_t client_len = sizeof(client_address);
            client_sockfd = accept(sockfd, (struct sockaddr*)&client_address, &client_len);

            if (client_sockfd < 0) {
                RCLCPP_WARN(this->get_logger(), "Waiting for TCP client connection...");
                return;  // waiting for client
            }
            client_connected = true;
            RCLCPP_INFO(this->get_logger(), "TCP client connected!");
        }

        if (!client_connected) return;

        // Using tf lookup to get the sensor data from TF
        geometry_msgs::msg::PoseStamped pose;
        if (send_tool_pose_) {
                if (tf_buffer_->canTransform("sensor_frame", "tool_frame", tf2::TimePoint(), tf2::durationFromSec(0.5))) {
                auto transform = tf_buffer_->lookupTransform("sensor_frame", "tool_frame", tf2::TimePoint(), tf2::durationFromSec(1.0));
                pose.header.stamp = this->now();
                pose.header.frame_id = "tool_sensor";
                pose.pose.position.x = transform.transform.translation.x;
                pose.pose.position.y = transform.transform.translation.y;
                pose.pose.position.z = transform.transform.translation.z;
                pose.pose.orientation = transform.transform.rotation;

                RCLCPP_DEBUG(this->get_logger(), "Sending TOOL pose");
            } else {                                                                                    // Sending zero to avoid corrupt or random data when there is no tf
                RCLCPP_WARN(this->get_logger(), "TF for TOOL not available. Please wait for the controller to start");
                pose.pose.position.x = 0.0;
                pose.pose.position.y = 0.0;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;  
            }
        } 
        else 
        {
            pose.header.stamp = this->now();
            pose.header.frame_id = "target_sensor";

            if (tf_buffer_->canTransform("sensor_frame", "target_frame", tf2::TimePoint(), tf2::durationFromSec(0.5))) {
                auto transform = tf_buffer_->lookupTransform("sensor_frame", "target_frame", tf2::TimePoint(), tf2::durationFromSec(1.0));
                pose.pose.position.x = transform.transform.translation.x;
                pose.pose.position.y = transform.transform.translation.y;
                pose.pose.position.z = transform.transform.translation.z;
                pose.pose.orientation = transform.transform.rotation;
                RCLCPP_DEBUG(this->get_logger(), "Sending TARGET pose");
            } 
            else {                                                                                         // Sending zero to avoid corrupt or random data when there is no tf
                RCLCPP_WARN(this->get_logger(), "TF for TARGET not available. Please wait for the controller to start");
                pose.pose.position.x = 0.0;
                pose.pose.position.y = 0.0;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;  
            }
        }

        rclcpp::SerializedMessage msg;
        serialization.serialize_message(&pose, &msg);

        write(client_sockfd,
        msg.get_rcl_serialized_message().buffer,
        msg.get_rcl_serialized_message().buffer_length);

        send_tool_pose_ = !send_tool_pose_;  // Sending alternatively
    }

}