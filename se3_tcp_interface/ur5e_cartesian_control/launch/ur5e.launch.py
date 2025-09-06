import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument , OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import NotSubstitution, AndSubstitution


def generate_launch_description():
    # ur5 stuff
    robot_name_ = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='combined_robot')
    robot_ip_ = LaunchConfiguration('robot_ip')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='172.22.22.2')  # IP address is important for ur5e, but here I have chosen a random valid ip address
    sensor_ip_ = LaunchConfiguration('sensor_ip')
    sensor_ip_arg = DeclareLaunchArgument('sensor_ip', default_value='127.0.0.1')
    headless_mode_ = LaunchConfiguration('headless_mode')
    headless_mode_arg = DeclareLaunchArgument('headless_mode', default_value='true')
    tool_device_name_ = LaunchConfiguration('tool_device_name')
    tool_device_name_arg = DeclareLaunchArgument('tool_device_name', default_value='/tmp/ttyUR')
    tool_tcp_port_ = LaunchConfiguration('tool_tcp_port')
    tool_tcp_port_arg = DeclareLaunchArgument('tool_tcp_port', default_value='54321')
    use_tool_communication_ = LaunchConfiguration('use_tool_communication')
    use_tool_communication_arg = DeclareLaunchArgument('use_tool_communication', default_value='false')
    tf_prefix_ = LaunchConfiguration('tf_prefix')
    tf_prefix_arg = DeclareLaunchArgument('tf_prefix', default_value='')
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )
    # fake gazebo or real
    use_fake_ = LaunchConfiguration('use_fake')
    use_fake_arg = DeclareLaunchArgument('use_fake', default_value='false')
    sim_ignition_ = LaunchConfiguration('use_gazebo')
    sim_ignition_arg = DeclareLaunchArgument('use_gazebo', default_value='false')
    fake_sensor_commands_ = LaunchConfiguration('fake_sensor_commands')
    fake_sensor_commands_arg = DeclareLaunchArgument('fake_sensor_commands', default_value='false')

    initial_joint_controller_ = LaunchConfiguration('initial_joint_controller')
    initial_joint_controller_arg = DeclareLaunchArgument('initial_joint_controller', default_value='scaled_joint_trajectory_controller')
    activate_joint_controller_ = LaunchConfiguration('activate_joint_controller')
    activate_joint_controller_arg = DeclareLaunchArgument('activate_joint_controller', default_value='true')
    
    # main controller yaml file - contains ur5, se3 and cartesian 
    ur5_controllers_path = os.path.join(get_package_share_directory('ur_robot_driver'),'config','ur_controllers.yaml')
    assignment9_controllers_path = os.path.join(get_package_share_directory('ur5e_cartesian_control'),'config','assignment9_controllers.yaml')

    
    # initial position file
    initial_position_file_path = os.path.join(get_package_share_directory('ur5e_cartesian_control'),'config','assignment9_initial_positions.yaml')
    initial_position_file_ = LaunchConfiguration('initial_position_file')
    initial_position_file_arg = DeclareLaunchArgument('initial_position_file', default_value=initial_position_file_path)
    
    # x y z
    target_x_ = LaunchConfiguration('x')
    target_x_arg = DeclareLaunchArgument('x', default_value='0.2')
    target_y_ = LaunchConfiguration('y')
    target_y_arg = DeclareLaunchArgument('y', default_value='-0.633')
    target_z_ = LaunchConfiguration('z')
    target_z_arg = DeclareLaunchArgument('z', default_value='0.687')
    world_file_ = LaunchConfiguration('world_file')
    world_file_arg = DeclareLaunchArgument('world_file', default_value='empty.sdf')
    use_sim_time_ = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    
    # common stuff
    # common_controllers_path = os.path.join(get_package_share_directory('ur5e_cartesian_control'),'config','ur_custom_controllers.yaml')
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "config",
             "ur5e_update_rate.yaml",
        ]
    )
    sensor_combined_path = os.path.join(get_package_share_directory("ur5e_cartesian_control"), "urdf", "ur5_with_sensor.urdf.xacro")
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', sensor_combined_path, 
                                                        ' name:=',
                                                        robot_name_,
                                                        ' robot_ip:=',
                                                        robot_ip_,
                                                        ' safety_limits:=',
                                                        "true",
                                                        ' safety_pos_margin:=',
                                                        "0.15",
                                                        ' safety_k_position:=',
                                                        "20",
                                                        ' ur_type:=',
                                                        "ur5e",
                                                        ' tf_prefix:=',
                                                        tf_prefix_,
                                                        ' headless_mode:=',
                                                        headless_mode_,
                                                        ' use_fake_hardware:=',
                                                        use_fake_,
                                                        ' fake_sensor_commands:=',
                                                        fake_sensor_commands_,
                                                        ' sim_ignition:=',
                                                        sim_ignition_,
                                                        ' initial_positions_file:=',
                                                        initial_position_file_,
                                                        " simulation_controllers:=",
                                                        assignment9_controllers_path,
                                                        " sensor_ip:=",
                                                        sensor_ip_,
                                                        " script_filename:=", 
                                                        script_filename,
                                                        " input_recipe_filename:=", 
                                                        input_recipe_filename,
                                                        " output_recipe_filename:=", 
                                                        output_recipe_filename,
                                                        ])
    
    combined_description = {"robot_description": robot_description_content}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time_}, combined_description],
    )
    
    tcp_server_node = Node(
        package="se3_sensor_driver",
        executable="tcp_server",
        name="tcp_server",
        output="screen"
    )

    # base to sensor
    static_tf_pub_base_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.5', '--y', '0.5', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll',
            '0', '--frame-id', 'base', '--child-frame-id', 'sensor_frame']
    )
    #flange to tool
    static_tf_pub_flange_tool = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.2', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll',
            '0', '--frame-id', 'flange', '--child-frame-id', 'tool_frame']
    )
    #sensor to target
    static_tf_pub_sensor_target = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_sensor_target',
        arguments=[
            target_x_, target_y_, target_z_, '0', '-0.7071', '0', '0.7071',         
            'sensor_frame', 'target_frame'
        ],
        output='screen'
    )


    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('ur5e_cartesian_control'), 'rviz', 'my_rviz.rviz')],
        parameters=[{"use_sim_time": use_sim_time_}],
        output='screen'
    )
    
    #Controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            combined_description,
            update_rate_config_file,
            ParameterFile(ur5_controllers_path, allow_substs=True),
            ParameterFile(assignment9_controllers_path, allow_substs=True)
        ],
        output="screen",
        condition=IfCondition(use_fake_)
    )

    delayed_fake_control_node_after_tcp = RegisterEventHandler(
        OnProcessStart(
            target_action=tcp_server_node,
            on_start=[
                TimerAction(
                    period=5.0, 
                    actions=[control_node],
                    condition=IfCondition(use_fake_)
                )
            ],
        )
    )


    ## FOR REAL ROBOT ONLY
    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            {"use_sim_time" : False},
            combined_description,
            update_rate_config_file,
            ParameterFile(ur5_controllers_path, allow_substs=True),
            ParameterFile(assignment9_controllers_path, allow_substs=True)
        ],
        output="screen",
        condition=IfCondition(AndSubstitution(NotSubstitution(sim_ignition_),NotSubstitution(use_fake_))),

    )
    

    delayed_real_control_node_after_tcp = RegisterEventHandler(
        OnProcessStart(
            target_action=tcp_server_node,
            on_start=[
                TimerAction(
                    period=5.0, 
                    actions=[ur_control_node],
                    condition=IfCondition(AndSubstitution(NotSubstitution(sim_ignition_),NotSubstitution(use_fake_)))
                )
            ],
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # For fake and real robot, I am starting controllers that the official package starts
    ur_controllers_active = [
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "ur_configuration_controller",
        # initial_joint_controller_,
    ]

    ur_controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tcp_pose_broadcaster",
    ]

    ur_core_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["--controller-manager", "/controller_manager", "--controller-manager-timeout", "10"] + ur_controllers_active,
    condition=UnlessCondition(sim_ignition_)
)

    ur_core_controller_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager", "/controller_manager", "--controller-manager-timeout", "10", "--inactive"] + ur_controllers_inactive,
        condition=UnlessCondition(sim_ignition_)
    )



    tool_pose_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tool_sensor_pose_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    target_pose_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["target_sensor_pose_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    cartesian_motion_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_motion_controller", "-c", "/controller_manager"],
        output="screen"
    )

    delay_pose_broadcasters = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[tool_pose_broadcaster, target_pose_broadcaster],
        )
    )

    delay_cartesian_motion_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=target_pose_broadcaster,
            on_exit=[cartesian_motion_controller],
        )
    )



    # ignition stuff
    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join
            (get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': [' -v 4 -r ', world_file_]}.items(),
            condition=IfCondition(sim_ignition_)
    )

    ign_create_node =  Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
        condition=IfCondition(sim_ignition_)
    )

    ign_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
        condition=IfCondition(sim_ignition_)
    )



    #Miscellaneous Nodes for real robot from ur_robot_driver
    real_robot_nodes = [
        Node(
            package="ur_robot_driver",
            executable="dashboard_client",
            name="dashboard_client",
            output="screen",
            emulate_tty=True,
            parameters=[{"robot_ip": robot_ip_}, {"use_sim_time" : False},],
            condition=IfCondition(AndSubstitution(NotSubstitution(sim_ignition_),NotSubstitution(use_fake_)))
        ),
        Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            name="ur_robot_state_helper",
            output="screen",
            parameters=[
                {"headless_mode": headless_mode_},
                {"robot_ip": robot_ip_},
                {"use_sim_time" : False},
            ],
            condition=IfCondition(AndSubstitution(NotSubstitution(sim_ignition_),NotSubstitution(use_fake_)))
        ),
        Node(
            package="ur_robot_driver",
            executable="tool_communication.py",
            name="ur_tool_comm",
            output="screen",
            parameters=[
                {
                    "robot_ip": robot_ip_,
                    "tcp_port": tool_tcp_port_,
                    "device_name": tool_device_name_,
                    "use_sim_time" : False,
                }
            ],
            condition=IfCondition(use_tool_communication_),
        ),
        Node(
            package="ur_robot_driver",
            executable="urscript_interface",
            parameters=[{"robot_ip": robot_ip_},{"use_sim_time" : False},],
            output="screen",
            condition=IfCondition(AndSubstitution(NotSubstitution(sim_ignition_),NotSubstitution(use_fake_)))
        ),
        Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"headless_mode": headless_mode_},
                {"joint_controller_active": activate_joint_controller_},
                {"use_sim_time" : False},
                {
                    "consistent_controllers": [
                        "io_and_status_controller",
                        "force_torque_sensor_broadcaster",
                        "joint_state_broadcaster",
                        "speed_scaling_state_broadcaster",
                        "tcp_pose_broadcaster",
                        "ur_configuration_controller",
                    ]
                },
            ],
            condition=IfCondition(AndSubstitution(NotSubstitution(sim_ignition_),NotSubstitution(use_fake_)))
        ),
    ]


    return LaunchDescription([
        robot_name_arg,
        robot_ip_arg,
        sensor_ip_arg,
        headless_mode_arg,
        use_fake_arg,
        fake_sensor_commands_arg,
        fake_sensor_commands_arg,
        use_tool_communication_arg,
        tool_tcp_port_arg,
        tool_device_name_arg,
        tf_prefix_arg,
        sim_ignition_arg, 
        activate_joint_controller_arg,
        world_file_arg,
        use_sim_time_arg,
        initial_position_file_arg,
        target_x_arg,
        target_y_arg,
        target_z_arg,
        initial_joint_controller_arg,
        *real_robot_nodes,
        static_tf_pub_base_sensor,
        static_tf_pub_flange_tool,
        static_tf_pub_sensor_target,
        tcp_server_node,
        robot_state_publisher_node,
        delayed_fake_control_node_after_tcp,
        delayed_real_control_node_after_tcp,
        joint_state_broadcaster_spawner,
        ur_core_controller_spawner,
        ur_core_controller_spawner_inactive,
        delay_pose_broadcasters,
        delay_cartesian_motion_controller,
        ign_gazebo_launch,
        ign_create_node,
        ign_bridge_node,
        rviz
        ])