#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <rclcpp/serialization.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace std;

namespace assignment8{
    class sensor_tcp_server : public rclcpp::Node {
        public:
            sensor_tcp_server (const std::string& name);
            ~sensor_tcp_server();
            void tf_lookup_timer();

        private:
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            int sockfd = -1;
            int client_sockfd = -1;
            bool client_connected = false;
            bool send_tool_pose_ = true;

            rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
            rclcpp::SerializedMessage tool_pose_msg_;
            rclcpp::SerializedMessage target_pose_msg_;
    };
}