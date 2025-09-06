# Assignment 9

It launches the complete package using single launch file. Commands:

Fake Mode:
```
ros2 launch ur5e_cartesian_control ur5e.launch.py use_fake:=true x:=somex y:=somey z:=somez
```

Ignition Gazebo Mode:
```
ros2 launch ur5e_cartesian_control ur5e.launch.py use_gazebo:=true x:=somex y:=somey z:=somez
```

Real Mode:
```
ros2 launch ur5e_cartesian_control ur5e.launch.py x:=somex y:=somey z:=somez
```



It works without x y and z also using default values. After a target is reached, the target can be moved using below commands, and the tool follows it

```
ros2 run tf2_ros static_transform_publisher 0.3 0.5 0.5 0.707 0 0.707 base target_frame
```