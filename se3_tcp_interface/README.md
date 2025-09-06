# Assignment 7

## se3_sensor_driver
Launch Command example 

`ros2 launch se3_sensor_driver sensor.launch.py`  

This will launch two pose broadcaster controllers for two sensors which can be visualised in rviz which is also launched once the controllers are activated(takes about 3-5 seconds). Topics can be echoed using following command

Tool Sensor Output - `ros2 topic echo /tool_sensor_pose_broadcaster/pose`   
Target Sensor Output - `ros2 topic echo /target_sensor_pose_broadcaster/pose` 


I noticed that the values printed in the hardware interface and the ones seen on /pose topic didn’t always match. After testing, I realized this was because the poses were randomly generated every second. Since the values change randomly and abruptly and the broadcaster reads them slightly later, the topic shows a different pose than what was printed. When I used fixed values or a smooth circular pose (not included in assignment) instead of random ones, the topic echoed the correct pose, which confirmed that everything is working as expected.

## Assignment 8 and 9 Update

I have updated the sensor interface to support zero or multiple sensors dynamically, without requiring any structural changes in the code.

I have combined both the tool and target sensors into a single hardware interface, SE3HardwareInterface.

For Ignition Gazebo, I ran into a limitation where Gazebo only accepts hardware interfaces that inherit from ign_ros2_control::IgnitionSystemInterface. Since my main sensor interface (SE3HardwareInterface) is derived from hardware_interface::SensorInterface, Ignition rejected it. To work around this, I added inheriotance from IgnitionSystemInterface.
Since I implemented a custom Cartesian controller that reads sensor data and commands robot joints, everything had to be accessible through the same controller manager (the one Gazebo launches).