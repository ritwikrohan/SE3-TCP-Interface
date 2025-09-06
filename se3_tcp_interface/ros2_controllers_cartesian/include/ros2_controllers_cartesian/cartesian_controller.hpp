#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>


#include <Eigen/Dense>

namespace assignment9_controller
{
    class CartesianMotionController : public controller_interface::ControllerInterface{

        public:

            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
            controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&);
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        protected:

            template <typename T> using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

            bool isActive() const { return active_; };

            bool initialized_ = {false};
            bool configured_ = {false};
            bool active_ = {false};

            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
            void targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr target);
            void writeJointControlCmds();
            KDL::Frame source_to_destination_tf (const std::string &source_link, const std::string &destination_link);
            geometry_msgs::msg::Pose get_sensor_data(const std::string &sensor_pose_name_);

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_cmd_handles_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_handles_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> sensor_tool_pose_handles_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> sensor_target_pose_handles_;
            std::vector<std::string> sensor_interfaces_;


            std::vector<std::string> joint_names_;
            std::string tool_pose_name_;
            std::string target_pose_name_;
            std::string cmd_interface_type_;      
            std::string end_effector_link_;     
            std::string robot_base_link_;      
            std::string robot_description_;
            std::string sensor_link_;
            std::string tool_link_;
            Eigen::VectorXd simulated_joint_cmd_;

            KDL::Chain chain_;
            std::shared_ptr<KDL::ChainIkSolverVel_wdls> inv_solver_;

            double motion_threshold_ = 1e-3;
            bool robot_stopped_ = true;

    };
}