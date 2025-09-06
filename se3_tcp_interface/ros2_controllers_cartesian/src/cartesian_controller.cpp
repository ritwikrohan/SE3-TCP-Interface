#include <ros2_controllers_cartesian/cartesian_controller.hpp>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/parameter.hpp>

namespace assignment9_controller
{

    controller_interface::CallbackReturn CartesianMotionController::on_init(){
        std::cout << "CartesianMotionController::on_init" << std::endl;
        if(!initialized_)
        {
            auto_declare<std::string>("robot_description", "");
            auto_declare<std::string>("sensor_link", "");
            auto_declare<std::string>("robot_base_link", "");
            auto_declare<std::string>("tool_link", "");
            auto_declare<std::string>("end_effector_link", "");
            auto_declare<std::string>("sensor_poses.tool_pose_name", "");
            auto_declare<std::string>("sensor_poses.target_pose_name", "");
            auto_declare<std::string>("interface_name", "");
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            sensor_interfaces_ = {"position.x", "position.y", "position.z","orientation.x", "orientation.y", "orientation.z", "orientation.w"}; // using this because ther is no predefined hardware interface for this
            initialized_ = true;
            robot_stopped_ = false;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CartesianMotionController::on_configure (const rclcpp_lifecycle::State&){

        std::cout << "CartesianMotionController::on_configure" << std::endl;

        if(configured_)
        { 
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; 
        }
        robot_description_ = get_node()->get_parameter("robot_description").as_string();
        std::cout << "Robot Description: "<< robot_description_ << std::endl;
        if(robot_description_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        sensor_link_ = get_node()->get_parameter("sensor_link").as_string();
        std::cout << "Sensor Link: "<< sensor_link_ << std::endl;
        if(sensor_link_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "sensor_link is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        robot_base_link_ = get_node()->get_parameter("robot_base_link").as_string();
        std::cout << robot_base_link_ << std::endl;
        if(robot_base_link_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        tool_link_ = get_node()->get_parameter("tool_link").as_string();
        std::cout << tool_link_ << std::endl;
        if(tool_link_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "tool_link is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        end_effector_link_ = get_node()->get_parameter("end_effector_link").as_string();
        std::cout << end_effector_link_ << std::endl;
        if(end_effector_link_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        tool_pose_name_ = get_node()->get_parameter("sensor_poses.tool_pose_name").as_string();
        std::cout << tool_pose_name_ << std::endl;
        if(tool_pose_name_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "sensor_poses.tool_pose_name is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        target_pose_name_ = get_node()->get_parameter("sensor_poses.target_pose_name").as_string();
        std::cout << target_pose_name_ << std::endl;
        if(target_pose_name_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "target_pose_name is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        joint_names_ = get_node()->get_parameter("joints").as_string_array();
        if(joint_names_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
            return controller_interface::CallbackReturn::ERROR;
        }
        for(const auto & joint_name : joint_names_)
        {
            std::cout << joint_name << std::endl;
        }


        cmd_interface_type_ = get_node()->get_parameter("interface_name").as_string();
        std::cout << cmd_interface_type_ << std::endl;
        if(cmd_interface_type_.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }


        KDL::Tree tree;
        if( kdl_parser::treeFromString(robot_description_, tree ) )
        {
            if( tree.getChain( robot_base_link_, end_effector_link_, chain_ ) )
            {
                inv_solver_ = std::make_shared<KDL::ChainIkSolverVel_wdls>(chain_);
            }
            else
            { 
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse chain");
            }
        }
        else
        { 
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse tree");
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        simulated_joint_cmd_.resize(joint_names_.size());
        simulated_joint_cmd_.setZero();

        target_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(get_node()->get_name() + std::string("/command"), 3, std::bind(&CartesianMotionController::targetCallback, this, std::placeholders::_1));
        configured_ = true;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CartesianMotionController::on_activate(const rclcpp_lifecycle::State&){

        if(active_) 
        { 
            return controller_interface::CallbackReturn::SUCCESS; 
        }

        if(!controller_interface::get_ordered_interfaces( command_interfaces_, joint_names_, cmd_interface_type_,joint_cmd_handles_ ))
        { 
            RCLCPP_ERROR(get_node()->get_logger(),"Expected %zu '%s' command interfaces, got %zu.", joint_names_.size(), cmd_interface_type_.c_str(),joint_cmd_handles_.size());
            return CallbackReturn::ERROR;
        }

        if(!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION, joint_state_handles_ )) 
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", joint_names_.size(), hardware_interface::HW_IF_POSITION,joint_state_handles_.size());
            return CallbackReturn::ERROR;
        }

        // https://docs.ros.org/en/rolling/p/controller_interface/generated/function_helpers_8hpp_1ad48120b7767321bba0453f34f498f3e2.html#exhale-function-helpers-8hpp-1ad48120b7767321bba0453f34f498f3e2
        // using full interface name according to the documentation above
        std::vector<std::string> tool_interface_names;
        std::vector<std::string> target_interface_names;
        for (const std::string &interface_name_ : sensor_interfaces_)
        {
            tool_interface_names.push_back(tool_pose_name_ + "/" + interface_name_);
            target_interface_names.push_back(target_pose_name_ + "/" + interface_name_);
        }

        if (!controller_interface::get_ordered_interfaces(state_interfaces_, tool_interface_names, "", sensor_tool_pose_handles_))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu tool pose interfaces, got %zu.", sensor_interfaces_.size(), sensor_tool_pose_handles_.size());
            return CallbackReturn::ERROR;
        }

        if (!controller_interface::get_ordered_interfaces(state_interfaces_, target_interface_names, "", sensor_target_pose_handles_))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu target pose interfaces, got %zu.", sensor_interfaces_.size(),sensor_target_pose_handles_.size());
            return CallbackReturn::ERROR;
        }
        active_ = true;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CartesianMotionController::on_deactivate (const rclcpp_lifecycle::State&){
        std::cout << "CartesianMotionController::on_deactivate" << std::endl;
        if(active_)
        { 
            joint_cmd_handles_.clear();
            joint_state_handles_.clear();
            sensor_tool_pose_handles_.clear();
            sensor_target_pose_handles_.clear();
            this->release_interfaces();
        }
        active_ = false;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // Only for UR5
    controller_interface::InterfaceConfiguration CartesianMotionController::command_interface_configuration() const{
            std::cout << "CartesianMotionController::command_interface_configuration" << std::endl;
            controller_interface::InterfaceConfiguration conf;
            conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
            conf.names.reserve(joint_names_.size() * cmd_interface_type_.size());
            for (const auto & joint_name : joint_names_)
            {
                conf.names.push_back(joint_name + std::string("/").append(cmd_interface_type_));
                std::cout << joint_name + std::string("/").append(cmd_interface_type_) << std::endl;
            }
            return conf;
    }
    // Both UR5 and SE3
    controller_interface::InterfaceConfiguration CartesianMotionController::state_interface_configuration() const{
        std::cout << "CartesianMotionController::state_interface_configuration" << std::endl;
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        conf.names.reserve(joint_names_.size()+(2*sensor_interfaces_.size()));
        for(const auto & joint_name : joint_names_){
            conf.names.push_back(joint_name + "/position");
            std::cout << joint_name + "/position" << std::endl;
        }
        for (const auto &interface : sensor_interfaces_)
        {
            conf.names.push_back(tool_pose_name_ + "/" + interface);
            std::cout << tool_pose_name_ + "/" + interface << std::endl;
        }

        for (const auto &interface : sensor_interfaces_)
        {
            conf.names.push_back(target_pose_name_ + "/" + interface);
            std::cout << target_pose_name_ + "/" + interface << std::endl;
        }
        return conf;

    }

    // Kept only for later use like in rsp github
    void CartesianMotionController::targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr /*target*/){
        if(!this->isActive())
        {
            return;
        }
    }

    controller_interface::return_type CartesianMotionController::update(const rclcpp::Time&, const rclcpp::Duration&){

        if (active_){
        geometry_msgs::msg::Pose tool_to_sensor_pose = get_sensor_data(tool_pose_name_);   // Obtaining tool pose from sensor's interface - transform (sensor-tool)
        KDL::Frame tool_to_sensor_kdl; 
        tf2::fromMsg(tool_to_sensor_pose, tool_to_sensor_kdl);
        KDL::Frame sensor_to_base_kdl = source_to_destination_tf(sensor_link_, robot_base_link_); // Used tf lookup to obtain the known data  - transform (sensor-base)
        KDL::Frame tool_to_base_kdl = sensor_to_base_kdl*tool_to_sensor_kdl;                                                               // - transform (sensor - tool - base)
        KDL::Frame flange_to_tool_kdl = source_to_destination_tf(end_effector_link_, tool_link_); // Used tf lookup to obtain the known data  - transform (tool - flange)
        KDL::Frame flange_to_base_kdl = tool_to_base_kdl*flange_to_tool_kdl;                                                               // - transform (flange - base) - using this for tool pose from base

        geometry_msgs::msg::Pose target_to_sensor_pose = get_sensor_data(target_pose_name_);  // Obtaining target pose from sensor's interface - transform (sensor - target)
        KDL::Frame target_to_sensor_kdl; 
        tf2::fromMsg(target_to_sensor_pose, target_to_sensor_kdl);                                                                        
        KDL::Frame target_to_base_kdl = sensor_to_base_kdl*target_to_sensor_kdl;                                                            // - transfomr (target - base) - using this for target pose from base

        // using https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1ChainIkSolverVel__wdls.html
        KDL::Twist frame_error_ = KDL::diff(flange_to_base_kdl, target_to_base_kdl);
        KDL::JntArray q_in_(joint_names_.size());
        for( std::size_t i=0; i<q_in_.rows(); i++ ){
            q_in_(i) = joint_state_handles_[i].get().get_value();
            // std::cout << q_in_(i) << std::setw(15);
        }
        
        // For debugging

        // std::cout << "\nTool (flange) position: " << tool_to_sensor_kdl.p.x() << ", "<< tool_to_sensor_kdl.p.y() << ", "<< tool_to_sensor_kdl.p.z() << std::endl;
        // std::cout << "Target position: "<< target_to_sensor_kdl.p.x() << ", "<< target_to_sensor_kdl.p.y() << ", "<< target_to_sensor_kdl.p.z() << std::endl;
        // std::cout << "Cartesian error (linear): "<< frame_error_.vel.x() << ", "<< frame_error_.vel.y() << ", "<< frame_error_.vel.z() << std::endl;
        // std::cout << "Cartesian error (angular): "<< frame_error_.rot.x() << ", "<< frame_error_.rot.y() << ", "<< frame_error_.rot.z() << std::endl;

        KDL::JntArray q_dot_out_(joint_names_.size());
        inv_solver_->CartToJnt(q_in_, frame_error_, q_dot_out_); // Used ik solver wdls

        for(std::size_t i=0; i<joint_names_.size(); ++i){
            simulated_joint_cmd_(i) = q_dot_out_(i);
            //  std::cout << q_dot_out_(i) << std::setw(15);
        }

        }
        writeJointControlCmds();
        return controller_interface::return_type::OK;
    }

    void CartesianMotionController::writeJointControlCmds()
    {
        bool moving_ = false;
        bool has_nan = false;
        bool has_singular = false;

        for (std::size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (i >= static_cast<std::size_t>(simulated_joint_cmd_.size())) continue;
            double velocity = simulated_joint_cmd_(i);
            if (std::isnan(velocity)) {
                has_nan = true;
            }
            if (std::abs(velocity) > 10.0) { 
                has_singular = true;
            }
            joint_cmd_handles_[i].get().set_value(0.3*velocity);

            if (std::abs(velocity) > motion_threshold_) {
                moving_ = true;
            }
        }
        if (has_nan) {
            RCLCPP_INFO(get_node()->get_logger(), "Some joint velocities are NaN");
        }
        if (has_singular) {
            RCLCPP_INFO(get_node()->get_logger(), "Possible singularity or collision. Emergency Stop!!");
        }

        if (moving_ && !robot_stopped_) {
            RCLCPP_INFO(get_node()->get_logger(), "Going to target.");
            robot_stopped_ = true;
        }
        else if (!moving_ && robot_stopped_) {
            RCLCPP_INFO(get_node()->get_logger(), "Reached target.");
            robot_stopped_ = false;
        }
    }




    // Uses parameter from the controller yaml to assign poses.  (Pose for sensor- tool and sensor - target) - Obtained from sensor interface
    geometry_msgs::msg::Pose CartesianMotionController::get_sensor_data(const std::string &sensor_pose_name_)
    {
        geometry_msgs::msg::Pose pose;

        if (sensor_pose_name_ == tool_pose_name_ && sensor_tool_pose_handles_.size() == sensor_interfaces_.size())
        {
            pose.position.x = sensor_tool_pose_handles_[0].get().get_value();
            pose.position.y = sensor_tool_pose_handles_[1].get().get_value();
            pose.position.z = sensor_tool_pose_handles_[2].get().get_value();
            pose.orientation.x = sensor_tool_pose_handles_[3].get().get_value();
            pose.orientation.y = sensor_tool_pose_handles_[4].get().get_value();
            pose.orientation.z = sensor_tool_pose_handles_[5].get().get_value();
            pose.orientation.w = sensor_tool_pose_handles_[6].get().get_value();
        }
        else if (sensor_pose_name_ == target_pose_name_ && sensor_target_pose_handles_.size() == sensor_interfaces_.size())
        {
            pose.position.x = sensor_target_pose_handles_[0].get().get_value();
            pose.position.y = sensor_target_pose_handles_[1].get().get_value();
            pose.position.z = sensor_target_pose_handles_[2].get().get_value();
            pose.orientation.x = sensor_target_pose_handles_[3].get().get_value();
            pose.orientation.y = sensor_target_pose_handles_[4].get().get_value();
            pose.orientation.z = sensor_target_pose_handles_[5].get().get_value();
            pose.orientation.w = sensor_target_pose_handles_[6].get().get_value();
        }
        else
        {
            RCLCPP_WARN(get_node()->get_logger(), "Sensor '%s' not recognized or handle size != %zu.", sensor_pose_name_.c_str(), sensor_interfaces_.size());
        }

        return pose;
    }

    // TF Lookup for known data (TF between sensor and robot base - calibration, TF between flange and tool)
    KDL::Frame CartesianMotionController::source_to_destination_tf (const std::string &source_link, const std::string &destination_link)
    {   
        KDL::Frame source_to_destination_kdl;
        geometry_msgs::msg::Pose pose;
        try {
            geometry_msgs::msg::TransformStamped transform_ = tf_buffer_->lookupTransform(source_link, destination_link, tf2::TimePointZero);
            transform_.transform.translation.x = pose.position.x;
            transform_.transform.translation.y = pose.position.y;
            transform_.transform.translation.z = pose.position.z;
            transform_.transform.rotation.x = pose.orientation.x;
            transform_.transform.rotation.y = pose.orientation.y;
            transform_.transform.rotation.z = pose.orientation.z;
            transform_.transform.rotation.w = pose.orientation.w;
            tf2::fromMsg(pose, source_to_destination_kdl);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_node()->get_logger(), "Transform failed: %s", ex.what());
        }
        return source_to_destination_kdl;
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(assignment9_controller::CartesianMotionController, controller_interface::ControllerInterface)