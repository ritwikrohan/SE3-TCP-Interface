import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

# Not used for assignment 8 and 9 - ur5e.launch.py has all in one

def generate_launch_description():

    controllers_config = os.path.join(get_package_share_directory("se3_sensor_driver"), "config", "se3_sensor.yaml")
    sensor_desc_path = os.path.join(get_package_share_directory("se3_sensor_driver"), "urdf", "se3_sensor.urdf.xacro")
    sensor_description = {"robot_description": Command([FindExecutable(name='xacro'), ' ', sensor_desc_path])}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[sensor_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[sensor_description, controllers_config],
        output="screen"
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

    tcp_server_node = Node(
        package="se3_sensor_driver",
        executable="tcp_server",
        name="tcp_server",
        output="screen"
    )


    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('se3_sensor_driver'), 'rviz', 'my_rviz.rviz')],
        output='screen'
    )

    delay_rviz_after_target_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=target_pose_broadcaster,
            on_exit=[rviz],
        )
    )

    return LaunchDescription([
        # tcp_server_node,
        control_node,
        robot_state_publisher_node,
        tool_pose_broadcaster,
        target_pose_broadcaster,
        delay_rviz_after_target_spawner,
        ])